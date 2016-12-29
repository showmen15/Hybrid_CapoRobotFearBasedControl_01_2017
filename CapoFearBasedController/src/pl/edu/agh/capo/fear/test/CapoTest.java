package pl.edu.agh.capo.fear.test;

import java.util.ArrayList;
import java.util.List;

import com.vividsolutions.jts.math.Vector2D;

import pl.edu.agh.capo.fear.CapoFearBasedController;
import pl.edu.agh.capo.fear.CapoFearBasedControllerMock;
import pl.edu.agh.capo.fear.CapoFearBasedControllerMock2;
import pl.edu.agh.capo.fear.SensorLoopMonitorThread;
import pl.edu.agh.capo.fear.app.CapoAppMock;

public class CapoTest
{

	public static RobotManager robotManager;

	public static void main(String[] args) throws Exception
	{
		String robotConfigPath = "D:/Desktop/Config.csv";
		String mapPath = "D:/Desktop/Map.json";

		RobotManager robot = new RobotManager(robotConfigPath, mapPath);

		// List<Vector2D> destinations = new ArrayList<>();
		// destinations.add(new Vector2D(2.0, 2.2));
		//
		// CapoFearBasedControllerMock2 temp2 = new
		// CapoFearBasedControllerMock2(Integer.parseInt("2"),
		// Double.parseDouble("0.5"), mapPath, 1.0, 1.0, -1.57, destinations);
		// Thread robot2Thread = new Thread(temp2);
		//
		// CapoFearBasedControllerMock2 temp = new
		// CapoFearBasedControllerMock2(Integer.parseInt("1"),
		// Double.parseDouble("0.5"), mapPath, 1.0, 1.0, -1.57, destinations);
		// Thread robot1Thread = new Thread(temp);
		//
		// robot1Thread.start();
		// robot2Thread.start();
		//
		// robot1Thread.join();
		// robot2Thread.join();

		int i = 323;

		// robotManager = new RobotManager("D:\\Desktop\\robots.csv",
		// "D:\\Desktop\\CapoRobotFearBasedControl_10_2015\\CommonResources\\MazeRoboLabEmptyMapWithGate.roson");

		// =
		// ;

		// runCount --;

		/*
		 * List<RobotController> robotControllers = new ArrayList<>(); try
		 * (BufferedReader br = new BufferedReader(new FileReader(confFile))) {
		 * String line; while ((line = br.readLine()) != null) { if
		 * (line.startsWith(COMMENT_SIGN)) { continue; } String[] robotData =
		 * line.split(COLUMN_SEPARATOR); if (robotData.length < 6) { continue; }
		 * RobotController robotController =
		 * createMockRobotController(Integer.parseInt(robotData[0]),
		 * EnvironmentalConfiguration.ROBOT_MAX_SPEED,
		 * Double.parseDouble(robotData[1]), mazeMap,
		 * parseDestinationList(robotData), robotManager);
		 * robotControllers.add(robotController); } } catch (IOException e) {
		 * e.printStackTrace(); } return robotControllers;
		 */
		// // jeden na jeden
		// String[] argsRobot1 = new String[] { "1", "0.3", mapPath, "1.1", "4",
		// "1", "1", "1", "1", "1.57", "111" };
		// String[] argsRobot2 = new String[] { "0", "0.3", mapPath, "1", "1",
		// "1", "4", "1", "4", "-1.57", "222" };

		// dwa na jeden
		// String[] argsRobot2 = new String[] { "0", "0.35", mapPath, "1.25",
		// "1", "1.25", "4", "1.25", "4", "-1.57", "222" };
		// String[] argsRobot1 = new String[] { "1", "0.2", mapPath, "1.25",
		// "4", "1.25", "1", "1.25", "1", "1.57", "111" };
		// String[] argsRobot3 = new String[] { "2", "0.2", mapPath, "0.75",
		// "4", "0.75", "1", "0.75", "1", "1.57", "111" };

		// dwa na dwa
		// String[] argsRobot2 = new String[] { "0", "0.35", mapPath, "1.25",
		// "1", "1.25", "4", "1.25", "4", "-1.57", "222" };
		// String[] argsRobot1 = new String[] { "1", "0.2", mapPath, "0.75",
		// "1", "0.75", "4", "0.75", "4", "-1.57", "111" };
		// String[] argsRobot3 = new String[] { "2", "0.2", mapPath, "1.25",
		// "4", "1.25", "1", "1.25", "1", "1.57", "111" };
		// String[] argsRobot4 = new String[] { "3", "0.3", mapPath, "0.75",
		// "4", "0.75", "1", "0.75", "1", "1.57", "222" };

		// start1_anagle caseID
		// String[] argsRobot1 = new String[] { "0", "0.3", mapPath, "1", "2",
		// "0", "0", "1", "0.8", "1.57", "111" };
		// String[] argsRobot4 = new String[] { "4", "0.3", mapPath, "1.7", "2",
		// "0", "0", "1.7", "0.8", "1.57", "444" };

		// String[] argsRobot1 = new String[] { "2", "0.3", mapPath, "1.25",
		// "4", "0", "0", "1.25", "0.7", "1.57", "111" };
		// String[] argsRobot2 = new String[] { "1", "0.3", mapPath, "1.25",
		// "3.5", "0", "0", "1.25", "1", "1.57", "222" };

		// String[] argsRobot1 = new String[] { "1", "0.3", mapPath, "1.25",
		// "4", "1.4", "1", "1.4", "1", "1.57", "111" };
		// String[] argsRobot2 = new String[] { "2", "0.3", mapPath, "1", "1",
		// "1.5", "4", "1.5", "4", "-1.57", "222" };

		// String[] argsRobot4 = new String[] { "4", "0.3", mapPath, "1.7", "4",
		// "1", "1", "1", "1", "1.57", "444" };

		// String[] argsRobot1 = new String[] { "1", "0.3", mapPath, "1.25",
		// "4", "0", "0", "1.4", "1", "1.57", "111" };
		// String[] argsRobot4 = new String[] { "4", "0.3", mapPath, "1.7", "4",
		// "0", "0", "1", "1", "1.57", "444" };

		// String[] argsRobot2 = new String[] { "2", "0.3", mapPath, "1", "1",
		// "1.5", "4", "1.5", "4", "-1.57", "222" };
		// String[] argsRobot3 = new String[] { "3", "0.3", mapPath, "1.5", "1",
		// "1", "4", "1", "4", "-1.57", "333" };

		// target1_x target1_y target2_x target2_y start1_x start1_y

		// String[] argsRobot1 = new String[] { "1", "0.3", mapPath, "1", "4",
		// "1", "1", "1.57", "111" };
		// String[] argsRobot2 = new String[] { "2", "0.3", mapPath, "1.5", "4",
		// "1.5", "1", "1.57", "111" };
		// String[] argsRobot3 = new String[] { "3", "0.3", mapPath, "2", "4",
		// "2", "1", "1.57", "222" };
		//
		// String[] argsRobot4 = new String[] { "0", "0.3", mapPath, "1.5", "1",
		// "1.5", "4", "-1.57", "222" };

		// String[] argsRobot1 = new String[] { "1", "0.2", mapPath, "1", "4",
		// "1", "1", "1.57", "111" };
		// String[] argsRobot3 = new String[] { "3", "0.2", mapPath, "2", "4",
		// "1.25", "1", "1.57", "222" };

		// String[] argsRobot1 = new String[] { "1", "0.3", mapPath, "1", "4",
		// "1", "3", "1.57", "111" };
		// String[] argsRobot2 = new String[] { "3", "0.3", mapPath, "1.6", "4",
		// "1.6", "3", "1.57", "222" };

		// String[] argsRobot2 = new String[] { "3", "0.3", mapPath, "1.6", "4",
		// "1.6", "1", "1.57", "222" };

		// String[] argsRobot1 = new String[] { "2", "0.3", mapPath, "1.25",
		// "4", "0.25", "1" };
		// String[] argsRobot2 = new String[] { "1", "0.35", mapPath, "1.25",
		// "1", "0.25", "1" };

		// String[] argsRobot2 = new String[] { "2", "0.3", mapPath, "1.15",
		// "4", "1.5", "1", "1.57", "111" };

		// String mapPath =
		// "D:\\Desktop\\CapoRobotFearBasedControl_10_2015\\CommonResources\\MazeRoboLabEmptyMapWithGate.roson";

		// String[] argsRobot1 = new String[] { "1", "0.3", mapPath, "1", "1",
		// "1", "1", "1", "4", "-1.57" };
		// String[] argsRobot2 = new String[] { "2", "0.3", mapPath, "1.5", "1",
		// "1.5", "1", "1.5", "4", "-1.57" };
		// String[] argsRobot3 = new String[] { "3", "0.3", mapPath, "2", "1",
		// "2", "1", "2", "4", "-1.57" };
		//
		// String[][] zbiorca = new String[][] { argsRobot1, argsRobot2,
		// argsRobot3 };
		//
		// List<Thread> tablica = new ArrayList<>();

		//
		// String[] argsRobot3 = new String[] { "3", "0.3", mapPath, "1.6", "4",
		// "1.6", "1", "1.57", "222" };
		//
		// String[] argsRobot4 = new String[] { "0", "0.3", mapPath, "1.15",
		// "1", "1.15", "3.7", "-1.57", "222" };
		//

		// TestCapoAppMock robot2 = new TestCapoAppMock(argsRobot1);
		// Thread robot2Thread = new Thread(robot2);
		//
		// TestCapoAppMock robot3 = new TestCapoAppMock(argsRobot2);
		// Thread robot3Thread = new Thread(robot3);
		//
		// TestCapoAppMock robot4 = new TestCapoAppMock(argsRobot1);
		// Thread robot4Thread = new Thread(robot4);
		//
		// robot2Thread.start();
		// robot3Thread.start();
		// robot4Thread.start();
		//
		// robot2Thread.join();
		// robot3Thread.join();
		// robot4Thread.join();

		// List<TestCapoAppMock> yyy = new ArrayList<>();
		//
		// for (int i = 0; i < zbiorca.length; i++)
		// {
		//
		// TestCapoAppMock robot1 = new TestCapoAppMock(zbiorca[i]);
		//
		// // yyy.add(robot1);
		// Thread robot1Thread = new Thread(robot1);
		//
		// tablica.add(robot1Thread);
		// }
		//
		// for (int i = 0; i < tablica.size(); i++)
		// {
		// tablica.get(i).start();
		// // temp.start();
		//
		// }
		// //
		// System.gc();
		//
		// tablica.get(0).join();
		// tablica.get(1).join();
		// tablica.get(2).join();
		//
		// //
		// // int i = 323;

		// TestCapoAppMock robot1 = new TestCapoAppMock(argsRobot1);
		// Thread robot1Thread = new Thread(robot1);
		//
		// TestCapoAppMock robot2 = new TestCapoAppMock(argsRobot2);
		// Thread robot2Thread = new Thread(robot2);
		//
		// TestCapoAppMock robot3 = new TestCapoAppMock(argsRobot3);
		// Thread robot3Thread = new Thread(robot3);
		//
		// TestCapoAppMock robot4 = new TestCapoAppMock(argsRobot4);
		// Thread robot4Thread = new Thread(robot4);
		//
		// robot1Thread.start();
		// robot2Thread.start();
		// // robot3Thread.start();
		// // robot4Thread.start();
		//
		// robot1Thread.join();
		// // robot2Thread.join();
		// // robot3Thread.join();
		// // robot4Thread.join();
	}
}