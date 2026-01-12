package frc.robot.shooter.training;

import frc.robot.shooter.data.ShotRecord;
import frc.robot.shooter.model.ModelConfig;

import java.io.FileWriter;
import java.util.List;

public class SimpleModelTrainer {

    public static void train(String csvPath, String outputJsonPath) {

        // Load training data
        List<ShotRecord> all = CSVReader.read(csvPath);
        List<ShotRecord> records = all.stream()
            .filter(r -> r.made) // or r.made == true
            .toList();

        int n = records.size();
        int features = 3; // distance, distance^2, bias

        double[][] X = new double[n][features];
        double[] y = new double[n];

        for (int i = 0; i < n; i++) {
            ShotRecord r = records.get(i);

            double d = r.distanceToHub;

            X[i][0] = d;        // linear term
            X[i][1] = d * d;    // quadratic term
            X[i][2] = 1.0;      // bias

            y[i] = r.shooterRPM;
        }

        // Solve for coefficients using least squares
        double[] coeffs = RegressionMath.solveLeastSquares(X, y);

        // Build model config
        ModelConfig config = new ModelConfig();
        config.a = coeffs[0]; // linear distance
        config.b = coeffs[1]; // quadratic distance^2
        config.g = coeffs[2]; // bias

        // Zero out unused real-world features
        config.c = 0;
        config.d = 0;
        config.e = 0;
        config.f = 0;

        // Write JSON
        try (FileWriter writer = new FileWriter(outputJsonPath)) {
            writer.write(config.toJson());
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }

    public static void main(String[] args) {

        String csvPath = "shooter_data.csv";
        String outputPath = "model.json";

        System.out.println("Training quadratic model from: " + csvPath);
        System.out.println("Writing model to: " + outputPath);

        train(csvPath, outputPath);

        System.out.println("Done.");
    }
}
