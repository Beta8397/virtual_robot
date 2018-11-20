package opmodelist;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;

public class OpModes {
    public static final ObservableList<String> opModes =
            FXCollections.observableArrayList(

                    "TestOpMode1",
                    "TestOpMode2", "TestOpMode3", "LineFollow"

            );
}
