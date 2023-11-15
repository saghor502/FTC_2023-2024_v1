package org.firstinspires.ftc.teamcode.domparser;

import java.io.FileWriter;
import java.io.IOException;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class DomParser {

    private static String directory;

    public DomParser(String directory){
        DomParser.directory = directory;
    }

    public static String writeData() throws JSONException {
        JSONObject positionDetails = new JSONObject();
        positionDetails.put("xPosition", "0");
        positionDetails.put("yPosition", "0");
        positionDetails.put("orientation", "0");

        JSONObject positionObjt = new JSONObject();
        positionObjt.put("position1", positionDetails);

        JSONObject positionDetails2 = new JSONObject();
        positionDetails2.put("xPosition", "0");
        positionDetails2.put("yPosition", "50");
        positionDetails2.put("orientation", "0");

        JSONObject positionObjt2 = new JSONObject();
        positionObjt2.put("position2", positionDetails);

        JSONObject positionDetails3 = new JSONObject();
        positionDetails3.put("xPosition", "0");
        positionDetails3.put("yPosition", "50");
        positionDetails3.put("orientation", "180");

        JSONObject positionObjt3 = new JSONObject();
        positionObjt3.put("position3", positionDetails);

        JSONArray positionDB = new JSONArray();
        try(FileWriter file = new FileWriter(directory)) {
            file.write(positionDB.toString());
            file.flush();
            return positionDB.toString();
        } catch (IOException e) {
            e.printStackTrace();
            return e.toString();
        }
    }
}
