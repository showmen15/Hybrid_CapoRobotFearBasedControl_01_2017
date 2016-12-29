package pl.edu.agh.capo.app;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

import com.google.gson.Gson;

import pl.edu.agh.capo.fear.communication.StateCollector;
import pl.edu.agh.capo.maze.MazeMap;
import pl.edu.agh.capo.ui.CapoMazeVisualizer;

public class FearMazeVisualize
{
	public static void main(String[] args)
	{
		if (args.length != 1)
			return;

		String mapPath = "./MapVisualizerFear.json";

		ConnectMSSQLServer log = new ConnectMSSQLServer();

		TaskConfig configure = log.GetTaskConfig(Integer.parseInt(args[0]));

		try
		{
			Files.write(Paths.get(mapPath), configure.Map.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING);
		}
		catch (IOException e1)
		{
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		MazeMap mazeMap;
		try
		{
			mazeMap = MazeMap.loadMazeFromFile(mapPath);
		}
		catch (Exception e)
		{
			e.printStackTrace();
			return;
		}
		System.out.println("Maze loaded");

		StateCollector collector = new StateCollector();
		collector.setConsumer(CapoMazeVisualizer.getInstance());

		CapoMazeVisualizer.getInstance().open(mazeMap);

	}

}
