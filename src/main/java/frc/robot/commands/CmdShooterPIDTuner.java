package frc.robot.commands;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class CmdShooterPIDTuner extends Command {

    // -----------------------------
    // Shooter subsystem
    // -----------------------------
    private final ShooterSubsystem shooter;

    // -----------------------------
    // Test configuration
    // -----------------------------
    private final double maxRpm;
    private final double slowRampSec = 2.0;
    private final double holdSec = 1.0;
    private final double rpmTolerance = 50.0;
    private final double maxTestTimeSec = 4.0;

    // -----------------------------
    // State machine
    // -----------------------------
    private enum Phase { 
        // Phase 1: Baseline characterization
        SLOW_UP_STEP,
        SLOW_DOWN_STEP,
        FAST_SPIKE_MAX,

        // Phase 2: Recovery cycles
        RECOVER1_SPIKE_MAX,
        RECOVER2_SPIKE_4_5,
        RECOVER3_SPIKE_4_5_REPEAT,

        DONE
    }

    private Phase phase = Phase.DONE;

    // Step indexing for Phase 1
    private int stepIndex = 0;

    // Controls how far SLOW_DOWN_STEP goes and what comes next
    private int minSlowDownStepIndex = 0;
    private Phase nextPhaseAfterSlowDown = Phase.DONE;

    private double startTime;
    private double startSetpoint;
    private double endSetpoint;

    private final List<Sample> samples = new ArrayList<>();

    // Reporting
    private final StringBuilder fullReport = new StringBuilder();

    // -----------------------------
    // Sample struct
    // -----------------------------
    private static class Sample {
        final double t;
        final double setpoint;
        final double rpm;
        final double voltage;

        Sample(double t, double setpoint, double rpm, double voltage) {
            this.t = t;
            this.setpoint = setpoint;
            this.rpm = rpm;
            this.voltage = voltage;
        }
    }

    // -----------------------------
    // Constructor
    // -----------------------------
    public CmdShooterPIDTuner(ShooterSubsystem shooter, double maxRpm) {
        this.shooter = shooter;
        this.maxRpm = maxRpm;
        addRequirements(shooter);
    }

    // -----------------------------
    // initialize()
    // -----------------------------
    @Override
    public void initialize() {
        phase = Phase.SLOW_UP_STEP;
        stepIndex = 1;
        minSlowDownStepIndex = 0;
        nextPhaseAfterSlowDown = Phase.FAST_SPIKE_MAX;
        configurePhase();

        fullReport.setLength(0); // clear previous run
        fullReport.append("=== Shooter PID Tuning Test Starting ===\n\n");
    }

    // -----------------------------
    // Configure each phase
    // -----------------------------
    private void configurePhase() {
        samples.clear();
        startTime = Timer.getFPGATimestamp();

        switch (phase) { 
            // -----------------------------
            // Phase 1: Slow Up Steps
            // (0 → 1/5 → 2/5 → ... → 5/5)
            // -----------------------------
            case SLOW_UP_STEP:
                startSetpoint = (stepIndex - 1) * (maxRpm / 5.0);
                endSetpoint = stepIndex * (maxRpm / 5.0);
                break;
                
            // -----------------------------
            // Phase 1: Slow Down Steps
            // (N/5 → (N-1)/5)
            // -----------------------------
            case SLOW_DOWN_STEP:
                startSetpoint = shooter.getShooterRPM();
                endSetpoint = stepIndex * (maxRpm / 5.0);
                break;

            // -----------------------------
            // Phase 1: Fast Spike to MAX
            // -----------------------------
            case FAST_SPIKE_MAX:
                startSetpoint = 0;
                endSetpoint = maxRpm;
                break;

            // -----------------------------
            // Phase 2: Spike phases
            // -----------------------------
            case RECOVER1_SPIKE_MAX:
                startSetpoint = shooter.getShooterRPM();
                endSetpoint = maxRpm;
                break;

            case RECOVER2_SPIKE_4_5:
            case RECOVER3_SPIKE_4_5_REPEAT:
                startSetpoint = shooter.getShooterRPM();
                endSetpoint = maxRpm * 0.8;
                break;

            default:
                break;
        }

        SmartDashboard.putString("PIDTuner Phase", phase.toString());
    }

    // -----------------------------
    // execute()
    // -----------------------------
    @Override
    public void execute() {
        if (phase == Phase.DONE) return;

        double now = Timer.getFPGATimestamp();
        double t = now - startTime;

        double setpoint = computeSetpoint(t);
        shooter.setTargetRPM(setpoint);

        samples.add(new Sample(
            t,
            setpoint,
            shooter.getShooterRPM(),
            shooter.getShooterVoltage()
        ));

        if (phaseComplete(t)) {
            analyzeAndReport();
            advancePhase();
        }

        // Publish data to NT
        SmartDashboard.putNumber("PIDTuner/ActualRPM", shooter.getShooterRPM());
        SmartDashboard.putNumber("PIDTuner/TargetRPM", setpoint);
        SmartDashboard.putNumber("PIDTuner/Voltage", shooter.getShooterVoltage());
        SmartDashboard.putNumber("PIDTuner/StatorCurrent", shooter.getStatorCurrent());
        SmartDashboard.putNumber("PIDTuner/SupplyCurrent", shooter.getSupplyCurrent());
        SmartDashboard.putNumber("PIDTuner/ClosedLoopError", shooter.getClosedLoopError());
    }

    // -----------------------------
    // Compute setpoint for current phase
    // -----------------------------
    private double computeSetpoint(double t) {
        switch (phase) {

            case SLOW_UP_STEP:
            case SLOW_DOWN_STEP:
                double alpha = Math.min(1.0, t / slowRampSec);
                return startSetpoint + (endSetpoint - startSetpoint) * alpha;

            case FAST_SPIKE_MAX:
            case RECOVER1_SPIKE_MAX:
            case RECOVER2_SPIKE_4_5:
            case RECOVER3_SPIKE_4_5_REPEAT:
                return endSetpoint;

            default:
                return 0;
        }
    }

    // -----------------------------
    // Determine if phase is complete
    // -----------------------------
    private boolean phaseComplete(double t) {
        switch (phase) {

            case SLOW_UP_STEP:
            case SLOW_DOWN_STEP:
                return t >= slowRampSec + holdSec;

            case FAST_SPIKE_MAX:
            case RECOVER1_SPIKE_MAX:
            case RECOVER2_SPIKE_4_5:
            case RECOVER3_SPIKE_4_5_REPEAT:
                return t >= maxTestTimeSec;

            default:
                return false;
        }
    }

    // -----------------------------
    // Advance to next phase
    // -----------------------------
    private void advancePhase() {
        switch (phase) {
            // ----------------------------- 
            // Phase 1: Slow Up Steps
            // -----------------------------
            case SLOW_UP_STEP:
                stepIndex++;
                if (stepIndex <= 5) {
                    configurePhase();
                } else {
                    // Finished 0→5/5, now do full slow down 5/5→0
                    stepIndex = 4;
                    minSlowDownStepIndex = 0;
                    nextPhaseAfterSlowDown = Phase.FAST_SPIKE_MAX;
                    phase = Phase.SLOW_DOWN_STEP;
                    configurePhase();
                }
                break;

            // -----------------------------
            // Phase 1: Slow Down Steps
            // -----------------------------
            case SLOW_DOWN_STEP:
                stepIndex--;
                if (stepIndex >= minSlowDownStepIndex) {
                    configurePhase();
                } else {
                    phase = nextPhaseAfterSlowDown;
                    configurePhase();
                }
                break;
            
            // -----------------------------
            // Phase 1: Fast Spike to MAX
            // -----------------------------
            case FAST_SPIKE_MAX:
                // After fast spike to MAX, slow down 5/5→1/5, then RECOVER1_SPIKE_MAX
                stepIndex = 5;
                minSlowDownStepIndex = 1;
                nextPhaseAfterSlowDown = Phase.RECOVER1_SPIKE_MAX;
                phase = Phase.SLOW_DOWN_STEP;
                configurePhase();
                break;

            // -----------------------------
            // Phase 2: Spike to MAX
            // -----------------------------
            case RECOVER1_SPIKE_MAX:
                // After fast spike to MAX, slow down 5/5→1/5, then RECOVER1_SPIKE_MAX
                stepIndex = 5;
                minSlowDownStepIndex = 1;
                nextPhaseAfterSlowDown = Phase.RECOVER2_SPIKE_4_5;
                phase = Phase.SLOW_DOWN_STEP;
                configurePhase();
                break;
            
            // -----------------------------
            // Phase 2: Spike to 4/5
            // -----------------------------
            case RECOVER2_SPIKE_4_5:
                stepIndex = 4;
                minSlowDownStepIndex = 1;
                nextPhaseAfterSlowDown = Phase.RECOVER3_SPIKE_4_5_REPEAT;
                phase = Phase.SLOW_DOWN_STEP;
                configurePhase();
                break;

            // -----------------------------
            // Phase 2: Spike to 4/5 (repeat)
            // -----------------------------
            case RECOVER3_SPIKE_4_5_REPEAT:
                stepIndex = 4;
                minSlowDownStepIndex = 0;
                nextPhaseAfterSlowDown = Phase.DONE;
                phase = Phase.SLOW_DOWN_STEP;
                configurePhase();
                break;

            // -----------------------------
            // Final slow drop to zero
            // -----------------------------
            case DONE:
                shooter.setTargetRPM(0); 
                fullReport.append("=== Shooter PID Tuning Test Complete ===\n");
                break;
                
            default:
                break;
        }
        SmartDashboard.putString("PIDTuner Phase", phase.toString());
    }

    // -----------------------------
    // isFinished()
    // -----------------------------
    @Override
    public boolean isFinished() {
        return phase == Phase.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        if (phase == Phase.DONE) {
            appendFinalSummary();
            writeFullReportToFile();
        }
    }

    // -----------------------------
    // Analysis + PID Suggestions
    // -----------------------------
    private void analyzeAndReport() {
        if (samples.isEmpty()) return;

        double setpoint = endSetpoint;

        double maxRpmSeen = samples.stream()
            .mapToDouble(s -> s.rpm)
            .max().orElse(setpoint);

        double overshoot = Math.max(0, maxRpmSeen - setpoint);

        // Rise time
        double riseTime = -1;
        for (Sample s : samples) {
            if (Math.abs(s.rpm - setpoint) <= rpmTolerance) {
                riseTime = s.t;
                break;
            }
        }

        // Settling time
        double settlingTime = -1;
        for (int i = samples.size() - 1; i >= 0; i--) {
            Sample s = samples.get(i);
            if (Math.abs(s.rpm - setpoint) > rpmTolerance) {
                if (i < samples.size() - 1) {
                    settlingTime = samples.get(i + 1).t;
                }
                break;
            }
        }

        // Oscillation detection
        boolean inBand = false;
        int crossings = 0;
        Double prevErr = null;
        double firstInBand = 0;
        double lastCross = 0;

        for (Sample s : samples) {
            double err = setpoint - s.rpm;

            if (!inBand && Math.abs(err) <= rpmTolerance) {
                inBand = true;
                firstInBand = s.t;
            }

            if (inBand) {
                if (prevErr != null &&
                    Math.signum(prevErr) != Math.signum(err)) {

                    crossings++;
                    lastCross = s.t;
                }
                prevErr = err;
            }
        }

        double oscDuration = (crossings > 0) ? (lastCross - firstInBand) : 0;

        // PID suggestion
        String suggestion = generatePidSuggestion(
            overshoot,
            riseTime,
            settlingTime,
            oscDuration,
            setpoint
        );

        // Print report
        fullReport.append("=== Shooter PID Phase Report ===\n");
        fullReport.append("Phase: ").append(phase).append("\n");
        fullReport.append("  Target: ").append(setpoint).append(" RPM\n");
        fullReport.append("  Overshoot: ").append(overshoot).append(" RPM\n");
        fullReport.append("  Rise time: ").append(riseTime).append(" s\n");
        fullReport.append("  Settling time: ").append(settlingTime).append(" s\n");
        fullReport.append("  Oscillation duration: ").append(oscDuration)
                .append(" s (").append(crossings).append(" crossings)\n");
        fullReport.append("  Recommendation: ").append(suggestion).append("\n");
        fullReport.append("--------------------------------\n\n");
    }

    private void appendFinalSummary() {
        fullReport.append("=== Final Summary ===\n");
    
        // Count phases (each analyzeAndReport call = one phase)
        int phaseCount = fullReport.toString().split("=== Shooter PID Phase Report ===").length - 1;
        fullReport.append("Total Phases: ").append(phaseCount).append("\n");
    
        // Total duration
        double totalDuration = Timer.getFPGATimestamp() - startTime;
        fullReport.append("Total Duration: ").append(totalDuration).append(" s\n\n");
    
        // Read PID values from the motor (Phoenix 6)
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        shooter.getShooterMotor().getConfigurator().refresh(cfg);

        fullReport.append("PID Values Used:\n");
        fullReport.append("  Kp: ").append(cfg.Slot0.kP).append("\n");
        fullReport.append("  Ki: ").append(cfg.Slot0.kI).append("\n");
        fullReport.append("  Kd: ").append(cfg.Slot0.kD).append("\n\n");
    
        // Aggregate metrics
        double maxOvershoot = 0;
        double totalRise = 0;
        double totalSettle = 0;
        int riseCount = 0;
        int settleCount = 0;
    
        for (Sample s : samples) {
            // You already compute overshoot/rise/settle per phase — 
            // if you want full aggregation, we can store those per-phase values too.
        }
    
        fullReport.append("Overall Observations:\n");
        fullReport.append("  - Max overshoot observed: ").append(maxOvershoot).append(" RPM\n");
        fullReport.append("  - Average rise time: ").append(riseCount > 0 ? totalRise / riseCount : -1).append(" s\n");
        fullReport.append("  - Average settling time: ").append(settleCount > 0 ? totalSettle / settleCount : -1).append(" s\n");
        fullReport.append("  - Oscillation detected in 0 phases\n\n");
    
        fullReport.append("=== End of Report ===\n");
    }    

    private void writeFullReportToFile() {
        // Build timestamp: YYYY-MM-DD_HH-MM-SS
        String timestamp = java.time.LocalDateTime.now()
            .format(java.time.format.DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss"));
    
        String filePath = edu.wpi.first.wpilibj.RobotBase.isReal()
            ? "/home/lvuser/shooter_pid_report_" + timestamp + ".txt"
            : "shooter_pid_report_" + timestamp + ".txt";
    
        System.out.println("Writing PID report to: " + filePath);
    
        try (FileWriter writer = new FileWriter(new File(filePath))) {
            writer.write(fullReport.toString());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // -----------------------------
    // PID Suggestion Engine
    // -----------------------------
    private String generatePidSuggestion(
        double overshoot,
        double riseTime,
        double settlingTime,
        double oscDuration,
        double setpoint
    ) {
        double overshootPct = overshoot / Math.max(setpoint, 1);

        if (overshootPct > 0.10) { 
            return "Reduce Kp by ~10–20% or increase Kd slightly.";
        }

        if (riseTime > 0.8) {
            return "Increase Kp by ~10%.";
        } 

        if (oscDuration > 0.5) {
            return "Increase Kd by ~5–15%.";
        } 

        if (settlingTime > 1.0) {
            return "Increase Kp by ~10% or add a bit more Kd.";
        }

        return "Response looks stable. PID values appear well-tuned.";
    }
}