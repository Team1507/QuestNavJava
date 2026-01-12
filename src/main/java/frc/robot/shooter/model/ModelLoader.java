package frc.robot.shooter.model;

import java.nio.file.Files;
import java.nio.file.Path;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.JsonNode;

import frc.robot.shooter.data.PoseSupplier;

public class ModelLoader {

    public static ShooterModel load(String path, PoseSupplier poseSupplier) {
        try {
            String json = Files.readString(Path.of(path));

            ObjectMapper mapper = new ObjectMapper();
            JsonNode root = mapper.readTree(json);

            ModelConfig config = new ModelConfig();
            config.a = root.get("a").asDouble();
            config.b = root.get("b").asDouble();
            config.c = root.get("c").asDouble();
            config.d = root.get("d").asDouble();
            config.e = root.get("e").asDouble();
            config.f = root.get("f").asDouble();
            config.g = root.get("g").asDouble();

            return new QuadraticShooterModel(config);

        } catch (Exception ex) {
            ex.printStackTrace();
            return new SimpleInterpolationModel();
        }
    }
}
