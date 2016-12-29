package pl.edu.agh.capo.fear.test;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

import com.google.gson.Gson;
import com.google.gson.JsonIOException;
import com.google.gson.JsonSyntaxException;

import pl.edu.agh.capo.fear.test.RobotManager;
import pl.edu.agh.capo.maze.MazeMap;

public class SingleRun
{

	public static TaskConfig configure;
	public static RobotManager robotManager;

	public static void main(String[] args) throws FileNotFoundException
	{

		if (args.length != 1)
			return;

		try
		{
			String robotConfigPath = "./Config.csv";
			String mapPath = "./Map.json";
			MazeMap mazeMap;

			ConnectMSSQLServer log = new ConnectMSSQLServer();

			configure = log.GetTaskConfig(Integer.parseInt(args[0]));

			Files.write(Paths.get(robotConfigPath), configure.ConfigFile.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING);

			Files.write(Paths.get(mapPath), configure.Map.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING);

			mazeMap = new Gson().fromJson(new FileReader(new File(mapPath)), MazeMap.class);

			File robotConfig = new File(robotConfigPath);

			robotManager = new RobotManager(robotConfigPath, mapPath);

		}
		catch (JsonSyntaxException | JsonIOException | IOException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
