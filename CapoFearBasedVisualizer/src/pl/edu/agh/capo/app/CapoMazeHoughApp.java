package pl.edu.agh.capo.app;

import java.io.FileNotFoundException;
import java.io.FileReader;

import com.google.gson.Gson;

import pl.edu.agh.capo.hough.CapoHough;
import pl.edu.agh.capo.maze.MazeMap;
import pl.edu.agh.capo.simulation.MeasureFileReader;

public class CapoMazeHoughApp {

	public static void main(String[] args) {


		Gson gson = new Gson();
		MazeMap mazeMap = null;
		try {
            mazeMap = gson.fromJson(new FileReader("F:/AGH/LabRobotow/CapoRobot/LokalizacjaJava/CapoMazeHough/res/MazeRoboLabFullMap2.roson" ), MazeMap.class);
        } catch (FileNotFoundException e1) {
            e1.printStackTrace();
            return;
        }
		
		System.out.println("Maze loaded");
		
		MeasureFileReader measureFileReader = new MeasureFileReader("F:/AGH/LabRobotow/CapoRobot/LokalizacjaJava/CapoMazeHough/res/DaneLabirynt1.csv");
		
		System.out.println("Data loaded");
		
		 CapoHough ch = new  CapoHough();
		 ch.getHoughLines(measureFileReader.next());
		
		

	}

}
