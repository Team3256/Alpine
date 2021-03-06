package auto.paths;

import frc.team3256.robot.auto.paths.JSONReader;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.junit.Test;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;
import java.util.stream.Collectors;

import static org.junit.Assert.assertEquals;

public class TestJSONReaderArrayList {
    @Test
    public void GetTranslationsArrayList() {

        String sampleText = "[\n" + "{\n" +
                "\"acceleration\": 10.0,\n" +
                "\"curvature\": 0.0,\n" +
                "\"pose\": {\n" +
                "\"rotation\": {\n" +
                "\"radians\": -0.23210383807515578\n" +
                "},\n" +
                "\"translation\": {    \n" +
                "\"x\": 123.30239583340617,\n" +
                "\"y\": 44.4397520790277\n" +
                "}\n" +
                "},\n" +
                "\"time\": 0.0,\n" +
                "\"velocity\": 0.0\n" +
                "},\n" +
                "{\n" +
                "\"acceleration\": 10.000000000000002,\n" +
                "\"curvature\": 0.00048125905761470765,\n" +
                "\"pose\": {\n" +
                "\"rotation\": {\n" +
                "\"radians\": -0.232077508168375\n" +
                "},\n" +
                "\"translation\": {\n" +
                "\"x\": 123.40866955812137,\n" +
                "\"y\": 44.414633820785355\n" +
                "}\n" +
                "},\n" +
                "\"time\": 0.1477848384602367,\n" +
                "\"velocity\": 1.4778483846023671\n" +
                "},\n" + "{\n" +
                "\"acceleration\": 10.0,\n" +
                "\"curvature\": 0.0,\n" +
                "\"pose\": {\n" +
                "\"rotation\": {\n" +
                "\"radians\": -0.23210383807515578\n" +
                "},\n" +
                "\"translation\": {    \n" +
                "\"x\": 123.50239583340617,\n" +
                "\"y\": 44.4597520790277\n" +
                "}\n" +
                "},\n" +
                "\"time\": 0.0,\n" +
                "\"velocity\": 0.0\n" +
                "},\n" +
                "{\n" +
                "\"acceleration\": 10.000000000000002,\n" +
                "\"curvature\": 0.00048125905761470765,\n" +
                "\"pose\": {\n" +
                "\"rotation\": {\n" +
                "\"radians\": -0.232077508168375\n" +
                "},\n" +
                "\"translation\": {\n" +
                "\"x\": 123.60866955812137,\n" +
                "\"y\": 44.54633820785355\n" +
                "}\n" +
                "},\n" +
                "\"time\": 0.1477848384602367,\n" +
                "\"velocity\": 1.4778483846023671\n" +
                "},\n" + "{\n" +
                "\"acceleration\": 10.0,\n" +
                "\"curvature\": 0.0,\n" +
                "\"pose\": {\n" +
                "\"rotation\": {\n" +
                "\"radians\": -0.23210383807515578\n" +
                "},\n" +
                "\"translation\": {    \n" +
                "\"x\": 123.10239583340617,\n" +
                "\"y\": 45.3397520790277\n" +
                "}\n" +
                "},\n" +
                "\"time\": 0.0,\n" +
                "\"velocity\": 0.0\n" +
                "},\n" +
                "{\n" +
                "\"acceleration\": 10.000000000000002,\n" +
                "\"curvature\": 0.00048125905761470765,\n" +
                "\"pose\": {\n" +
                "\"rotation\": {\n" +
                "\"radians\": -0.232077508168375\n" +
                "},\n" +
                "\"translation\": {\n" +
                "\"x\": 125.40866955812137,\n" +
                "\"y\": 44.414633820785355\n" +
                "}\n" +
                "},\n" +
                "\"time\": 0.1477848384602367,\n" +
                "\"velocity\": 1.4778483846023671\n" +
                "},\n" + "{\n" +
                "\"acceleration\": 10.0,\n" +
                "\"curvature\": 0.0,\n" +
                "\"pose\": {\n" +
                "\"rotation\": {\n" +
                "\"radians\": -0.23210383807515578\n" +
                "},\n" +
                "\"translation\": {    \n" +
                "\"x\": 123.54239583340617,\n" +
                "\"y\": 44.4397520790277\n" +
                "}\n" +
                "},\n" +
                "\"time\": 0.0,\n" +
                "\"velocity\": 0.0\n" +
                "},\n" +
                "{\n" +
                "\"acceleration\": 10.000000000000002,\n" +
                "\"curvature\": 0.00048125905761470765,\n" +
                "\"pose\": {\n" +
                "\"rotation\": {\n" +
                "\"radians\": -0.232077508168375\n" +
                "},\n" +
                "\"translation\": {\n" +
                "\"x\": 123.98766955812137,\n" +
                "\"y\": 44.414633820785355\n" +
                "}\n" +
                "},\n" +
                "\"time\": 0.1477848384602367,\n" +
                "\"velocity\": 1.4778483846023671\n" +
                "},\n" + "{\n" +
                "\"acceleration\": 10.0,\n" +
                "\"curvature\": 0.0,\n" +
                "\"pose\": {\n" +
                "\"rotation\": {\n" +
                "\"radians\": -0.23210383807515578\n" +
                "},\n" +
                "\"translation\": {    \n" +
                "\"x\": 123.30239583340617,\n" +
                "\"y\": 44.4297520790277\n" +
                "}\n" +
                "},\n" +
                "\"time\": 0.0,\n" +
                "\"velocity\": 0.0\n" +
                "},\n" +
                "{\n" +
                "\"acceleration\": 10.000000000000002,\n" +
                "\"curvature\": 0.00048125905761470765,\n" +
                "\"pose\": {\n" +
                "\"rotation\": {\n" +
                "\"radians\": -0.232077508168375\n" +
                "},\n" +
                "\"translation\": {\n" +
                "\"x\": 123.40866955812137,\n" +
                "\"y\": 44.994633820785355\n" +
                "}\n" +
                "},\n" +
                "\"time\": 0.1477848384602367,\n" +
                "\"velocity\": 1.4778483846023671\n" +
                "},\n" + "{\n" +
                "\"acceleration\": 10.0,\n" +
                "\"curvature\": 0.0,\n" +
                "\"pose\": {\n" +
                "\"rotation\": {\n" +
                "\"radians\": -0.23210383807515578\n" +
                "},\n" +
                "\"translation\": {    \n" +
                "\"x\": 123.11230239583340617,\n" +
                "\"y\": 44.4927520790277\n" +
                "}\n" +
                "},\n" +
                "\"time\": 0.0,\n" +
                "\"velocity\": 0.0\n" +
                "},\n" +
                "{\n" +
                "\"acceleration\": 10.000000000000002,\n" +
                "\"curvature\": 0.00048125905761470765,\n" +
                "\"pose\": {\n" +
                "\"rotation\": {\n" +
                "\"radians\": -0.232077508168375\n" +
                "},\n" +
                "\"translation\": {\n" +
                "\"x\": 139.40866955812137,\n" +
                "\"y\": 45.414633820785355\n" +
                "}\n" +
                "},\n" +
                "\"time\": 0.1477848384602367,\n" +
                "\"velocity\": 1.4778483846023671\n" +
                "},\n" + "{\n" +
                "\"acceleration\": 10.0,\n" +
                "\"curvature\": 0.0,\n" +
                "\"pose\": {\n" +
                "\"rotation\": {\n" +
                "\"radians\": -0.23210383807515578\n" +
                "},\n" +
                "\"translation\": {    \n" +
                "\"x\": 139.30239583340617,\n" +
                "\"y\": 44.9897520790277\n" +
                "}\n" +
                "},\n" +
                "\"time\": 0.0,\n" +
                "\"velocity\": 0.0\n" +
                "},\n" +
                "{\n" +
                "\"acceleration\": 10.000000000000002,\n" +
                "\"curvature\": 0.00048125905761470765,\n" +
                "\"pose\": {\n" +
                "\"rotation\": {\n" +
                "\"radians\": -0.232077508168375\n" +
                "},\n" +
                "\"translation\": {\n" +
                "\"x\": 123.90866955812137,\n" +
                "\"y\": 44.914633820785355\n" +
                "}\n" +
                "},\n" +
                "\"time\": 0.1477848384602367,\n" +
                "\"velocity\": 1.4778483846023671\n" +
                "},\n" + "]";

        JSONArray sampleArray = new JSONArray();
        JSONParser obj = new JSONParser();
        try {
            sampleArray = (JSONArray) obj.parse(sampleText);
        } catch (ParseException e) {
            e.printStackTrace();
        }
        ArrayList<Vector<Double>> inputList = JSONReader.ParseJSONFileTester(sampleArray);

        ArrayList<Vector<Double>> compareWith = new ArrayList<Vector<Double>>();

        Vector<Double> coord1 = new Vector<Double>();
        Vector<Double> coord2 = new Vector<Double>();
        Vector<Double> coord3 = new Vector<Double>();
        Vector<Double> coord4 = new Vector<Double>();
        Vector<Double> coord5 = new Vector<Double>();
        Vector<Double> coord6 = new Vector<Double>();
        Vector<Double> coord7 = new Vector<Double>();
        Vector<Double> coord8 = new Vector<Double>();

        coord1.add(0, 123.30239583340617);
        coord1.add(1, 44.4397520790277);
        compareWith.add(coord1);

        coord2.add(0, 123.10239583340617);
        coord2.add(1, 45.3397520790277);
        compareWith.add(coord2);

        coord3.add(0, 125.40866955812137);
        coord3.add(1, 44.414633820785355);
        compareWith.add(coord3);

        coord4.add(0, 123.54239583340617);
        coord4.add(1, 44.4397520790277);
        compareWith.add(coord4);

        // modify here
        coord5.add(0, 123.40866955812137);
        coord5.add(1, 44.994633820785355);
        compareWith.add(coord5);

        coord6.add(0, 123.11230239583340617);
        coord6.add(1, 44.4927520790277);
        compareWith.add(coord6);

        coord7.add(0, 139.40866955812137);
        coord7.add(1, 45.414633820785355);
        compareWith.add(coord7);

        coord8.add(0, 123.90866955812137);
        coord8.add(1, 44.914633820785355);
        compareWith.add(coord8);

        assertEquals(compareWith, inputList);
    }
}