package time;

public class ElapsedTime {

    private volatile long nsStartTime;

    public ElapsedTime(){
        nsStartTime = System.nanoTime();
    }

    public void reset(){
        nsStartTime = System.nanoTime();
    }

    public double milliseconds(){
        return (double)(System.nanoTime() - nsStartTime) / 1000000.0;
    }

    public double seconds(){
        return (double)(System.nanoTime() - nsStartTime) / 1000000000.0;
    }

    public long nanoseconds(){
        return System.nanoTime() - nsStartTime;
    }
}
