package MotorControllers;

import java.util.ArrayList;

/**
 * Created by robotics on 8/17/17.
 */

/*
    ~~~
 */
//TODO: ask what this does
public class FiFoBuffer {
    private ArrayList buffer;
    private int length = 0;

    FiFoBuffer(int l){
        buffer = new <Long>ArrayList();
        length = l;
        //Log.d("Buffer Size: ", Integer.toString(l));
    }

    public void add(long toAdd){
        Long l = new Long(toAdd);
        buffer.add(l);
        //Log.d("Buffer add: " , Long.toString(toAdd));
        //Log.d("Buffer length ", Long.toString(buffer.size()));
        if(buffer.size() >= length){
            buffer.remove(0);
        }
        //Log.d("Buffer new length ", Long.toString(buffer.size()));
    }
    public long getLast(){
        Long toReturn = (Long) buffer.get(0);
        long val = toReturn.longValue();
        //Log.d("Get Last return ", Long.toString(val));
        buffer.remove(0);
        return toReturn;
    }


}
