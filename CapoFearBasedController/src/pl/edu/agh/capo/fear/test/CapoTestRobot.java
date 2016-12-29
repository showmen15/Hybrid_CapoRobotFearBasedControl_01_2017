package pl.edu.agh.capo.fear.test;

import com.vividsolutions.jts.math.Vector2D;

import pl.edu.agh.capo.fear.CapoFearBasedController;
import pl.edu.agh.capo.fear.CapoFearBasedControllerMock;
import pl.edu.agh.capo.fear.SensorLoopMonitorThread;
import pl.edu.agh.capo.fear.app.CapoAppMock;

public class CapoTestRobot
{

	public static void main(String[] args) throws Exception
	{
		String mapPath = "D:\\Desktop\\CapoRobotFearBasedControl_10_2015\\CommonResources\\MazeRoboLabEmptyMapWithGate.roson";

		String[] argsRobot1 = new String[] { "1", "0.4", mapPath, "1.25", "0.7", "0.25", "1" };
		String[] argsRobot2 = new String[] { "2", "0.4", mapPath, "1.25", "4", "0.25", "1" };

		String[] argsRobot3 = new String[] { "3", "0.4", mapPath, "2", "4", "2", "1" };
		String[] argsRobot4 = new String[] { "0", "0.4", mapPath, "1.15", "1", "1.15", "3.7" };

		TestCapoAppRobot robot1 = new TestCapoAppRobot(argsRobot1);
		Thread robot1Thread = new Thread(robot1);

		TestCapoAppRobot robot2 = new TestCapoAppRobot(argsRobot2);
		Thread robot2Thread = new Thread(robot2);

		TestCapoAppRobot robot3 = new TestCapoAppRobot(argsRobot3);
		Thread robot3Thread = new Thread(robot3);

		TestCapoAppRobot robot4 = new TestCapoAppRobot(argsRobot4);
		Thread robot4Thread = new Thread(robot4);

		robot1Thread.start();
		robot2Thread.start();

		// robot3Thread.start();
		// robot4Thread.start();

		robot1Thread.join();
		robot2Thread.join();
		robot3Thread.join();
		robot4Thread.join();
	}
}