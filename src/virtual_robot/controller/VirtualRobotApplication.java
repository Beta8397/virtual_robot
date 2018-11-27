package virtual_robot.controller;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;

public class VirtualRobotApplication extends Application {

    private static VirtualRobotController controllerHandle;

    @Override
    public void start(Stage primaryStage) throws Exception{
        FXMLLoader loader = new FXMLLoader(getClass().getResource("virtual_robot.fxml"));
        Parent root = (BorderPane)loader.load();
        //Parent root = (BorderPane)FXMLLoader.load(getClass().getResource("virtual_robot.fxml"));
        primaryStage.setTitle("Virtual Robot");
        primaryStage.setScene(new Scene(root));
        primaryStage.setResizable(false);
        primaryStage.show();
        controllerHandle = loader.getController();
    }

    @Override
    public void stop() {
        if (controllerHandle.executorService != null && !controllerHandle.executorService.isShutdown()) {
            controllerHandle.executorService.shutdown();
        }
    }


    public static void main(String[] args) {
        launch(args);
    }
}
