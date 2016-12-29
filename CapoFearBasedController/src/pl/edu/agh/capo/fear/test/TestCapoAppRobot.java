package pl.edu.agh.capo.fear.test;

import java.io.IOException;

import pl.edu.agh.capo.fear.CapoFearBasedController;
import pl.edu.agh.capo.fear.CapoFearBasedControllerMock;
import pl.edu.agh.capo.fear.SensorLoopMonitorThread;

import com.vividsolutions.jts.math.Vector2D;

public class TestCapoAppRobot implements Runnable
{
	private String[] args;

	public TestCapoAppRobot(String[] args1) throws IOException
	{
		args = args1;
	}

	public void run()
	{
		System.out.println("\n This will loop CapoFearBasedController between 2 targets \n");
		if (args.length != 7)
		{
			System.out.println("\n Params: robotId maxVelocity mazeFilePath target1_x target1_y target2_x target2_y  \n\n\n");
			return;
		}
		System.out.println("\n\n\n hit 'p,Enter' for pause, 'x,Enter' to stop and terminate. \n\n\n");

		// "F:/AGH/LabRobotow/CapoRobot/LokalizacjaJava/CapoMazeHough/res/MazeRoboLabFullMap2.roson"
		CapoFearBasedController capoController;
		try
		{
			capoController = new CapoFearBasedController(Integer.parseInt(args[0]), Double.parseDouble(args[1]), args[2], 0);
			capoController.setLoopDestinations(new Vector2D(Double.parseDouble(args[3]), Double.parseDouble(args[4])), new Vector2D(Double.parseDouble(args[5]), Double.parseDouble(args[6])));
		}
		catch (Exception e)
		{
			e.printStackTrace();
			return;
		}

		SensorLoopMonitorThread sensorLoopMonitorThread = new SensorLoopMonitorThread(capoController);

		Thread monitorThread = new Thread(sensorLoopMonitorThread);
		capoController.SetMonitorThread(monitorThread);
		Thread controllerThread = new Thread(capoController);

		controllerThread.start();
		monitorThread.start();

		int c = 0;

		while (true)
		{

			try
			{
				c = System.in.read();
			}
			catch (IOException e)
			{
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			if (c == 'x')
				break;
			if (c == 'p')
				capoController.togglePaused();
		}
		System.out.println("STOP");
		sensorLoopMonitorThread.Stop();
		capoController.Stop();
		try
		{
			controllerThread.join();
		}
		catch (InterruptedException localInterruptedException)
		{
		}
		capoController.Stop();
	}

}
