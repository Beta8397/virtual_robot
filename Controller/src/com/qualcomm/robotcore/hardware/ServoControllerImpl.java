package com.qualcomm.robotcore.hardware;

public class ServoControllerImpl implements ServoController{
    private static ServoControllerImpl theInstance = null;
    private ServoControllerImpl() {}
    public static ServoControllerImpl getInstance(){
        if (theInstance == null) {
            theInstance = new ServoControllerImpl();
        }
        return theInstance;
    }
}
