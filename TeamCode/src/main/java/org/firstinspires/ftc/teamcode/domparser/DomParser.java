package org.firstinspires.ftc.teamcode.domparser;

import org.firstinspires.ftc.teamcode.odometry.position.Position;
import org.w3c.dom.Document;
import org.xml.sax.SAXException;

import java.io.IOException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

public class DomParser {

    private static String directory;

    public DomParser(String directory){
        DomParser.directory = directory;
    }

    public static Position getPosition(String id) throws ParserConfigurationException {
        double xPosition = 0;
        double yPosition = 0;
        int orientation = 0;

        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        try{
            DocumentBuilder builder = factory.newDocumentBuilder();
            Document doc = builder.parse(directory);
            xPosition = Double.parseDouble(doc.getElementById(id).getAttribute("xPosition"));
            xPosition = Double.parseDouble(doc.getElementById(id).getAttribute("yPosition"));
            xPosition = Double.parseDouble(doc.getElementById(id).getAttribute("orientation"));
        } catch(ParserConfigurationException e){
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (SAXException e) {
            e.printStackTrace();
        }


        Position position = new Position(xPosition, yPosition, orientation);
        return position;
    }

    public static void setPosition(String id, Position position) throws ParserConfigurationException {
        double xPosition = position.getXPosition();
        double yPosition = position.getYPosition();
        int orientation = position.getOrientation();

        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        try{
            DocumentBuilder builder = factory.newDocumentBuilder();
            Document doc = builder.parse(directory);
            doc.getElementById(id).setAttribute("xPosition", String.valueOf(xPosition));
            doc.getElementById(id).setAttribute("yPosition", String.valueOf(yPosition));
            doc.getElementById(id).setAttribute("orientation", String.valueOf(orientation));
        } catch(ParserConfigurationException e){
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (SAXException e) {
            e.printStackTrace();
        }
    }
}
