package pl.edu.agh.capo.fear.test;

import pl.edu.agh.capo.fear.CapoFearBasedControllerMock2;
import pl.edu.agh.capo.maze.MazeMap;
import pl.edu.agh.capo.maze.Node;
import pl.edu.agh.capo.maze.NodeNode;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.stream.Collectors;

import com.vividsolutions.jts.math.Vector2D;

public class RobotManager implements IRobotManager
{
	public static final int MOVE_ROBOT_PERIOD_IN_MS = 200;

	public static final String ROBOT_MAX_SPEED = "0.5";

	private static final String COMMENT_SIGN = "#";
	private static final String COLUMN_SEPARATOR = ";";

	// private static final int RUN_COUNT = 30;
	private final String confFilePath;
	private final String mazeMap;

	// private int runCount;
	private List<Thread> robots;

	private Map<Integer, Integer> result;

	public RobotManager(String confFilePath1, String mazeMapPath)
	{
		this.confFilePath = confFilePath1;
		this.mazeMap = mazeMapPath;
		result = new TreeMap<>();
		restart();
	}

	private void restart()
	{
		result.clear();
		robots = new ArrayList<>();

		String mapPath = mazeMap;
		String RobotID;
		String target1_x;
		String target1_y;
		String target2_x;
		String target2_y;
		String start1_x;
		String start1_y;
		String start1_anagle;
		List<Destination> destinationList;

		try (BufferedReader br = new BufferedReader(new FileReader(confFilePath)))
		{
			String line;
			while ((line = br.readLine()) != null)
			{
				if (line.startsWith(COMMENT_SIGN))
				{
					continue;
				}

				String[] robotData = line.split(COLUMN_SEPARATOR);

				if (robotData.length < 6)
				{
					continue;
				}

				destinationList = parseDestinationList(robotData);
				RobotID = robotData[0];

				start1_x = Double.toString(destinationList.get(0).X);
				start1_y = Double.toString(destinationList.get(0).Y);
				start1_anagle = robotData[1];

				target1_x = target2_x = Double.toString(destinationList.get(destinationList.size() - 1).X);
				target1_y = target2_y = Double.toString(destinationList.get(destinationList.size() - 1).Y);

				String[] argsRobot = new String[] { RobotID, ROBOT_MAX_SPEED, mapPath, target1_x, target1_y, target2_x, target2_y, start1_x, start1_y, start1_anagle };

				// TestCapoAppMock temp = new TestCapoAppMock(argsRobot);

				List<Vector2D> destinations = new ArrayList<>();

				for (int i = 1; i < destinationList.size(); i++)
				{
					Destination temp = destinationList.get(i);

					destinations.add(new Vector2D(temp.X, temp.Y));
				}

				CapoFearBasedControllerMock2 temp;
				try
				{
					temp = new CapoFearBasedControllerMock2(Integer.parseInt(RobotID), Double.parseDouble(ROBOT_MAX_SPEED), mapPath, destinationList.get(0).X, destinationList.get(0).Y, Double.parseDouble(start1_anagle), destinations);

					temp.robotManger = this;

					robots.add(new Thread(temp));
				}
				catch (Exception e)
				{
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
		catch (FileNotFoundException e1)
		{
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		catch (IOException e1)
		{
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		for (int i = 0; i < robots.size(); i++)
			robots.get(i).start();

		try
		{
			robots.get(0).join();
		}
		catch (InterruptedException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
	public void onFinish(int id, int time)
	{
		result.put(id, time);
		restartIfNeeded();
	}

	private void restartIfNeeded()
	{
		if (result.size() == robots.size())
		{
			printResult();
		}
	}

	private void printResult()
	{
		ConnectMSSQLServer log = new ConnectMSSQLServer();

		StringBuilder sb = new StringBuilder();
		for (int id : result.keySet())
		{

			log.SaveResult(SingleRun.configure, id, result.get(id), result.get(id) * MOVE_ROBOT_PERIOD_IN_MS);

			sb.append(String.format("%d;%d;%d\n", id, result.get(id), result.get(id) * MOVE_ROBOT_PERIOD_IN_MS));
		}
		System.out.print(sb.toString());
		System.exit(1);

	}

	private static List<Destination> parseDestinationList(String[] robotData)
	{
		int index = 2;
		List<Destination> destinationList = new ArrayList<>();
		while (index < robotData.length - 2)
		{
			double margin = Double.parseDouble(robotData[index]);
			double x = Double.parseDouble(robotData[index + 1]);
			double y = Double.parseDouble(robotData[index + 2]);
			destinationList.add(new Destination(x, y));
			index += 3;
		}
		return destinationList;
	}

}