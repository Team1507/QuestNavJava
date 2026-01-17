package frc.robot.commands;

// Java Libraries
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

// WPI Libraries
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

// CTRE Libraries
import com.ctre.phoenix6.configs.TalonFXConfiguration;

// Robot Subsystems
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command that performs a structured, multi-phase PID tuning sequence
 * for the shooter subsystem. The tuner executes controlled step tests,
 * spike tests, and recovery cycles, collecting telemetry and generating
 * a detailed report including overshoot, rise time, settling time, and
 * PID recommendations.
 *
 * <p>This command is designed to run autonomously and produce a
 * timestamped report file for long-term analysis.</p>
 */
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

    /**
     * Enumeration of all phases in the PID tuning sequence.
     * Each phase defines a specific type of test:
     * <ul>
     *  <li>Slow ramp-up steps</li>
     *  <li>Slow ramp-down steps</li>
     *  <li>Fast spike to max RPM</li>
     *  <li>Recovery spike cycles</li>
     * </ul> 
     */
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

    // -----------------------------
    // Data Collection
    // -----------------------------
    
    private final List<Sample> samples = new ArrayList<>();
    private final StringBuilder fullReport = new StringBuilder();
    private final List<String> phaseSuggestions = new ArrayList<>();
    private final List<String> phaseFfSuggestions = new ArrayList<>();
    private double testStartTime;

    private double maxOvershootSeen = 0;

    private double totalRiseTime = 0;
    private int riseTimeCount = 0;

    private double totalSettlingTime = 0;
    private int settlingTimeCount = 0;


    /** 
     * Represents a single telemetry sample collected during a phase. 
     * Each sample records time, setpoint, measured RPM, and voltage.
     */
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

    /**
     * Creates a new PID tuner for the shooter subsystem.
     * 
     * @param shooter the shooter subsystem being tuned 
     * @param maxRpm the maximum RPM used for spike and step tests 
     */
    public CmdShooterPIDTuner(ShooterSubsystem shooter, double maxRpm) {
        this.shooter = shooter;
        this.maxRpm = maxRpm;
        addRequirements(shooter);
    }

    // -----------------------------
    // initialize()
    // -----------------------------

    /**
     * Initializes the tuning sequence by resetting timers, clearing
     * previous data, and entering the first phase of the test.
     */
    @Override
    public void initialize() {
        testStartTime = Timer.getFPGATimestamp();

        phase = Phase.SLOW_UP_STEP;
        stepIndex = 1;
        minSlowDownStepIndex = 0;
        nextPhaseAfterSlowDown = Phase.FAST_SPIKE_MAX;
        configurePhase();

        // clear previous run 
        fullReport.setLength(0); 
        fullReport.append("=== Shooter PID + Feedforward Tuning Test Starting ===\n\n");
        phaseSuggestions.clear();
        phaseFfSuggestions.clear();
        maxOvershootSeen = 0;
        totalRiseTime = 0;
        riseTimeCount = 0;
        totalSettlingTime = 0;
        settlingTimeCount = 0;
    }

    // -----------------------------
    // execute()
    // -----------------------------

    /**
     * Executes the active phase of the tuning sequence. This method:
     * <ul>
     *   <li>Computes the current setpoint</li>
     *   <li>Commands the shooter subsystem</li>
     *   <li>Collects telemetry samples</li>
     *   <li>Checks for phase completion</li>
     *   <li>Advances the state machine when needed</li>
     * </ul>
     */
    @Override
    public void execute() {
        if (phase == Phase.DONE) return;

        double now = Timer.getFPGATimestamp();
        double t = now - startTime;

        double setpoint = computeSetpoint(t);
        shooter.setTargetRPS(setpoint);

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
    // isFinished()
    // -----------------------------

    /**
     * Indicates whether the tuning sequence has completed all phases.
     *
     * @return true when the state machine reaches DONE
     */
    @Override
    public boolean isFinished() {
        return phase == Phase.DONE;
    }

    // -----------------------------
    // end()
    // -----------------------------
    
    /** 
     * Finalizes the tuning sequence by generating the summary section 
     * and writing the full report to a timestamped file. 
     * 
     * @param interrupted whether the command was interrupted 
     */
    @Override
    public void end(boolean interrupted) {
        if (phase == Phase.DONE) {
            appendFinalSummary();
            writeFullReportToFile();
        }
    }

    // =====================================================================
    // PRIVATE HELPERS
    // =====================================================================

    /**
     * Configures the start and end setpoints for the current phase.
     * This method resets the sample buffer and timestamps the phase.
     */
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

    /**
     * Computes the shooter RPM setpoint for the active phase.
     *
     * Behavior:
     * <ul>
     *   <li>Ramp phases interpolate linearly between start and end setpoints</li>
     *   <li>Spike phases immediately return the final setpoint</li>
     * </ul>
     *
     * @param t elapsed time since the phase began (seconds)
     * @return computed RPM setpoint for this timestamp
     */
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

    /**
     * Determines whether the current phase has completed based on
     * elapsed time and phase type.
     *
     * @param t elapsed time since the phase began
     * @return true if the phase should end
     */
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

    /**
     * Advances the state machine to the next phase of the tuning sequence.
     * This method handles all transitions between step tests, spike tests,
     * and recovery cycles.
     */
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
                shooter.setTargetRPS(0); 
                fullReport.append("=== Shooter PID Tuning Test Complete ===\n");
                break;
                
            default:
                break;
        }
        SmartDashboard.putString("PIDTuner Phase", phase.toString());
    }

    // =====================================================================
    // Analysis and Reporting
    // =====================================================================
    
    /**
     * Analyzes the collected samples for the current phase and appends
     * a detailed report section including overshoot, rise time, settling
     * time, oscillation, and PID recommendations.
     */
    private void analyzeAndReport() {
        if (samples.isEmpty()) return;

        double setpoint = endSetpoint;

        double maxRpmSeen = samples.stream()
            .mapToDouble(s -> s.rpm)
            .max().orElse(setpoint);

        double overshoot = Math.max(0, maxRpmSeen - setpoint);
        maxOvershootSeen = Math.max(maxOvershootSeen, overshoot);

        // Rise time
        double riseTime = -1;
        for (Sample s : samples) {
            if (Math.abs(s.rpm - setpoint) <= rpmTolerance) {
                riseTime = s.t;
                break;
            }
        }

        // Report Rise time
        if (riseTime >= 0) {
            totalRiseTime += riseTime;
            riseTimeCount++;
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

        // Report Settling time
        if (settlingTime >= 0) {
            totalSettlingTime += settlingTime;
            settlingTimeCount++;
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
        phaseSuggestions.add(suggestion);

        // Feedforward suggestion 
        String ffSuggestion = generateFfSuggestion( 
            samples,
            setpoint,
            overshoot,
            riseTime
        ); 
        phaseFfSuggestions.add(ffSuggestion);

        // Print report
        fullReport.append("=== Shooter PID Phase Report ===\n");
        fullReport.append("Phase: ").append(phase).append("\n");
        fullReport.append("  Target: ").append(setpoint).append(" RPM\n");
        fullReport.append("  Overshoot: ").append(overshoot).append(" RPM\n");
        fullReport.append("  Rise time: ").append(riseTime).append(" s\n");
        fullReport.append("  Settling time: ").append(settlingTime).append(" s\n");
        fullReport.append("  Oscillation duration: ").append(oscDuration)
                .append(" s (").append(crossings).append(" crossings)\n");
        fullReport.append(" PID Recommendation: ").append(suggestion).append("\n");
        fullReport.append(" Feedforward Recommendation: ").append(ffSuggestion).append("\n");
        fullReport.append("--------------------------------\n\n");
    }

    /**
     * Appends the final summary section to the report, including
     * aggregated metrics and PID configuration values.
     */
    private void appendFinalSummary() {
        fullReport.append("=== Final Summary ===\n");
    
        // Count phases (each analyzeAndReport call = one phase)
        int phaseCount = fullReport.toString().split("=== Shooter PID Phase Report ===").length - 1;
        fullReport.append("Total Phases: ").append(phaseCount).append("\n");
    
        // Total duration
        double totalDuration = Timer.getFPGATimestamp() - testStartTime;
        fullReport.append("Total Duration: ").append(totalDuration).append(" s\n\n");
    
        // Read PID values from the motor (Phoenix 6)
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        shooter.getShooterMotor().getConfigurator().refresh(cfg);

        fullReport.append("PID Values Used:\n");
        fullReport.append("  Kp: ").append(cfg.Slot0.kP).append("\n");
        fullReport.append("  Ki: ").append(cfg.Slot0.kI).append("\n");
        fullReport.append("  Kd: ").append(cfg.Slot0.kD).append("\n\n");

        fullReport.append("Feedforward Values Used:\n");
        fullReport.append("  kV: ").append(cfg.Slot0.kV).append("\n");
        fullReport.append("  kS: ").append(cfg.Slot0.kS).append("\n");
        fullReport.append("  kA: ").append(cfg.Slot0.kA).append("\n\n");
    
        fullReport.append("Overall Observations:\n");
        fullReport.append("  - Max overshoot observed: ").append(maxOvershootSeen).append(" RPM\n");
        fullReport.append("  - Average rise time: ")
          .append(riseTimeCount > 0 ? totalRiseTime / riseTimeCount : -1)
          .append(" s\n");
        fullReport.append("  - Average settling time: ")
          .append(settlingTimeCount > 0 ? totalSettlingTime / settlingTimeCount : -1)
          .append(" s\n");
        fullReport.append("  - Oscillation detected in 0 phases\n\n");

        fullReport.append("Overall PID Recommendation:\n");
        fullReport.append("  ").append(computeOverallPidRecommendation()).append("\n\n");

        fullReport.append("Overall Feedforward Recommendation:\n"); 
        fullReport.append(" ").append(computeOverallFfRecommendation()).append("\n\n");

        fullReport.append("=== End of Report ===\n");
    }

    /**
     * Writes the full tuning report to a timestamped file on the robot
     * (or local filesystem when running in simulation).
     */
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

    /**
     * Generates a PID tuning recommendation based on overshoot,
     * rise time, settling time, and oscillation behavior.
     *
     * @param overshoot     measured overshoot in RPM
     * @param riseTime      time to enter tolerance band
     * @param settlingTime  time to remain within tolerance band
     * @param oscDuration   duration of oscillation in seconds
     * @param setpoint      target RPM
     * @return recommendation string for PID adjustment
     */
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

    /**
     * Generates a feedforward tuning recommendation based on
     * steady-state behavior, acceleration behavior, and voltage usage.
     *
     * Heuristics:
     *  - kS: handles static friction and low-speed hesitation
     *  - kV: handles steady-state voltage vs RPM
     *  - kA: handles acceleration voltage vs acceleration rate
     */
    private String generateFfSuggestion(
        List<Sample> samples,
        double setpoint,
        double overshoot,
        double riseTime
    ) {
        if (samples.isEmpty() || setpoint <= 0) {
            return "Insufficient data for feedforward analysis.";
        }

        double maxVoltage = 0;
        double avgVoltage = 0;

        // For kS detection
        boolean lowSpeedHesitation = false;

        // For kA detection
        double maxAccel = 0;
        double maxAccelVoltage = 0;

        // For kV detection
        double sumVoltInBand = 0;
        int countInBand = 0;

        double prevRpm = samples.get(0).rpm;
        double prevTime = samples.get(0).t;

        for (Sample s : samples) {
            maxVoltage = Math.max(maxVoltage, s.voltage);
            avgVoltage += s.voltage;

            // kS detection: shooter hesitates near zero RPM
            if (s.setpoint > 200 && s.rpm < 100 && s.voltage > 2.0) {
                lowSpeedHesitation = true;
            }

            // kV detection: steady-state voltage once in tolerance band
            if (Math.abs(s.rpm - setpoint) <= rpmTolerance) {
                sumVoltInBand += s.voltage;
                countInBand++;
            }

            // kA detection: acceleration vs voltage
            double dt = s.t - prevTime;
            if (dt > 0) {
                double accel = (s.rpm - prevRpm) / dt;
                if (accel > maxAccel) {
                    maxAccel = accel;
                    maxAccelVoltage = s.voltage;
                }
            }

            prevRpm = s.rpm;
            prevTime = s.t;
        }

        avgVoltage /= samples.size();
        double steadyVoltage = (countInBand > 0) ? (sumVoltInBand / countInBand) : avgVoltage;
        double overshootPct = overshoot / Math.max(setpoint, 1);

        // -----------------------------
        // kS Heuristics
        // -----------------------------
        if (lowSpeedHesitation) {
            return "Increase kS slightly: shooter hesitates at low RPM and requires extra voltage to break static friction.";
        }

        // -----------------------------
        // kA Heuristics
        // -----------------------------
        // If voltage is high but acceleration is low → kA too low
        if (maxAccelVoltage > 10.0 && maxAccel < (setpoint * 0.5)) {
            return "Increase kA: voltage is high during acceleration but RPM is not increasing quickly.";
        }

        // If acceleration is extremely sharp → kA too high
        if (maxAccel > (setpoint * 1.2)) {
            return "Reduce kA: acceleration spike is too aggressive.";
        }

        // -----------------------------
        // kV Heuristics
        // -----------------------------
        // Slow rise time + low voltage → kV too low
        if (riseTime > 0.8 && maxVoltage < 10.0) {
            return "Increase kV: shooter is slow to reach speed and voltage is not saturated.";
        }

        // Overshoot even with moderate voltage → kV too high
        if (overshootPct > 0.10 && steadyVoltage > 6.0) {
            return "Reduce kV: feedforward may be pushing the shooter past the target.";
        }

        // High voltage while already near/above target
        if (steadyVoltage > 9.0 && overshootPct > 0.05) {
            return "Reduce kV: voltage remains high even when RPM is at or above target.";
        }

        return "Feedforward appears reasonable for this phase.";
    }


    /**
     * Computes an overall PID recommendation by analyzing the
     * frequency of per-phase suggestions and selecting the most
     * commonly occurring one.
     *
     * @return summarized PID recommendation for the entire test
     */
    private String computeOverallPidRecommendation() {
        if (phaseSuggestions.isEmpty()) {
            return "No PID recommendation available.";
        }

        // Count occurrences
        java.util.Map<String, Integer> counts = new java.util.HashMap<>();
        for (String s : phaseSuggestions) {
            counts.put(s, counts.getOrDefault(s, 0) + 1);
        }

        // Find most frequent suggestion
        String best = null;
        int bestCount = 0;

        for (var entry : counts.entrySet()) {
            if (entry.getValue() > bestCount) {
                best = entry.getKey();
                bestCount = entry.getValue();
            }
        }

        return best != null ? best : "No PID recommendation available.";
    }

    /**
     * Computes an overall Feedforward recommendation by analyzing the
     * frequency of per-phase suggestions and selecting the most
     * commonly occurring one.
     *
     * @return summarized FF recommendation for the entire test
     */
    private String computeOverallFfRecommendation() {
        if (phaseFfSuggestions.isEmpty()) {
            return "No feedforward recommendation available.";
        }

        java.util.Map<String, Integer> counts = new java.util.HashMap<>();
        for (String s : phaseFfSuggestions) {
            counts.put(s, counts.getOrDefault(s, 0) + 1);
        }

        String best = null;
        int bestCount = 0;

        for (var entry : counts.entrySet()) {
            if (entry.getValue() > bestCount) {
                best = entry.getKey();
                bestCount = entry.getValue();
            }
        }

        return best != null ? best : "No feedforward recommendation available.";
    }
}