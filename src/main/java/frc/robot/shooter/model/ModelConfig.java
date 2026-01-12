package frc.robot.shooter.model;

public class ModelConfig {

    public double a; // distance
    public double b; // stator current
    public double c; // supply current
    public double d; // closed-loop error
    public double e; // pressDuration
    public double f; // shotsInBurst
    public double g; // bias

    public String toJson() {
        return "{\n" +
            "  \"a\": " + a + ",\n" +
            "  \"b\": " + b + ",\n" +
            "  \"c\": " + c + ",\n" +
            "  \"d\": " + d + ",\n" +
            "  \"e\": " + e + ",\n" +
            "  \"f\": " + f + ",\n" +
            "  \"g\": " + g + "\n" +
            "}";
    }
}
