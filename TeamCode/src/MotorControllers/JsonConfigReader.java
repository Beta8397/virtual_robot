package MotorControllers;

import Misc.Log;

import com.google.gson.JsonParser;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;

import Autonomous.Location;

/**
 * Created by Jeremy on 8/7/2017.
 */

/*
    A class to help us more easily use .json files with java
 */
public class JsonConfigReader {
    private JsonParser parser;
    private JSONObject jsonObject;

    public JsonConfigReader(InputStream stream){
        parser = new JsonParser();
        try{
            jsonObject = parseJSONData(stream);
        }
        catch(Exception e){
            //TODO implement something
        }
    }
    private JSONObject parseJSONData(InputStream inputStream) {
        String JSONString = null;
        JSONObject JSONObject = null;
        try {
            int sizeOfJSONFile = inputStream.available();
            byte[] bytes = new byte[sizeOfJSONFile];
            inputStream.read(bytes);
            inputStream.close();
            JSONString = new String(bytes, "UTF-8");
            JSONObject = new JSONObject(JSONString);
        } catch (IOException ex) {
             return null;
        }
        catch (JSONException x) {
            x.printStackTrace();
            return null;
        }
        return JSONObject;
    }
    public String getString(String n) throws Exception {
        return (String) jsonObject.get(n);
    }

    public int getInt(String n) throws Exception {
        return jsonObject.getInt(n);
    }

    public double getDouble(String n) throws Exception {
        return jsonObject.getDouble(n);
    }

    public long getLong(String n) throws Exception {
        return jsonObject.getLong(n);
    }

    public boolean getBoolean(String n) throws Exception {
        return jsonObject.getBoolean(n);
    }

    public Location getLocation(String n) throws Exception {
        String locToParse = jsonObject.getString(n);
        Location toReturn = parseLocation(locToParse);
        return toReturn;
    }

    public Location[] getPath(String n) throws Exception {
        ArrayList<Location> locationList = new ArrayList<Location>();
        String pathToParse = jsonObject.getString(n);
        int curIndex = 0;
        while (pathToParse.indexOf("(", curIndex) >= curIndex) {
            Log.d("Substring", pathToParse.substring(pathToParse.indexOf("(", curIndex), pathToParse.indexOf(")", curIndex)+1));
            String locToParse = pathToParse.substring(pathToParse.indexOf("(", curIndex), pathToParse.indexOf(")", curIndex)+1);
            locationList.add(parseLocation(locToParse));
            curIndex = pathToParse.indexOf(")", curIndex) + 1;
        }
        Location[] path = new Location[locationList.size()];
        path = locationList.toArray(path);
        return path;
    }

    private Location parseLocation(String n) {
        String subX = n.substring(n.indexOf("(") + 1, n.indexOf(","));
        String subY = n.substring(n.indexOf(",") + 1, n.indexOf(")"));
        double x = Double.parseDouble(subX);
        double y = Double.parseDouble(subY);
        Log.d("X: " + x," Y: " + y);
        Location toReturn = new Location(x, y);
        return toReturn;
    }
}
