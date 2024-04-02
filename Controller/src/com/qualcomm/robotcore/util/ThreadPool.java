package com.qualcomm.robotcore.util;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class ThreadPool {
    public static ExecutorService newFixedThreadPool(int i, String name) {
        return Executors.newFixedThreadPool(i);
    }
}
