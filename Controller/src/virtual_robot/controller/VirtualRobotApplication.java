package virtual_robot.controller;

import javafx.application.Application;
import javafx.event.EventHandler;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;

public class VirtualRobotApplication extends Application {

    private static VirtualRobotController controllerHandle;

    @Override
    public void start(Stage primaryStage) throws Exception{
        FXMLLoader loader = new FXMLLoader(getClass().getResource("virtual_robot.fxml"));
        Parent root = (BorderPane)loader.load();
        controllerHandle = loader.getController();
        primaryStage.setTitle("Virtual Robot");
        primaryStage.setScene(new Scene(root));
        primaryStage.setResizable(false);
        primaryStage.setOnShowing(new EventHandler<WindowEvent>() {
            @Override
            public void handle(WindowEvent event) {
                controllerHandle.setConfig(null);
            }
        });
        primaryStage.show();
    }

    @Override
    public void stop() {
        if (controllerHandle.executorService != null && !controllerHandle.executorService.isShutdown()) {
            controllerHandle.executorService.shutdown();
        }
    }

    public static VirtualRobotController getControllerHandle(){return controllerHandle;}


    public static void main(String[] args) {
        launch(args);
    }
}
