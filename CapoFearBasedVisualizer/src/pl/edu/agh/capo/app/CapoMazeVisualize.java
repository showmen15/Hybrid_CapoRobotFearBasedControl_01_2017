package pl.edu.agh.capo.app;

import pl.edu.agh.capo.fear.communication.StateCollector;
import pl.edu.agh.capo.maze.MazeMap;
import pl.edu.agh.capo.ui.CapoMazeVisualizer;

public class CapoMazeVisualize
{

	public static void main(String[] args)
	{
		if (args.length != 1)
		{
			System.out.println("\n Params: mazeFilePath\n\n\n");
			return;
		}

		MazeMap mazeMap;
		try
		{
			mazeMap = MazeMap.loadMazeFromFile(args[0]);
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
