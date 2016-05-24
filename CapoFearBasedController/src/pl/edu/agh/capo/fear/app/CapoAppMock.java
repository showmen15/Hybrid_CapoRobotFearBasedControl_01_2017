package pl.edu.agh.capo.fear.app;

import com.vividsolutions.jts.math.Vector2D;

import pl.edu.agh.capo.fear.CapoFearBasedController;
import pl.edu.agh.capo.fear.CapoFearBasedControllerMock;
import pl.edu.agh.capo.fear.SensorLoopMonitorThread;

public class CapoAppMock
{
	public static void main(String[] args) throws Exception
	{
		System.out.println("\n This will loop CapoFearBasedController between 2 targets \n");
		if (args.length != 11)
		{
			System.out.println("\n Params: robotId maxVelocity mazeFilePath target1_x target1_y target2_x target2_y start1_x start1_y start1_anagle  \n\n\n");
			return;
		}
		System.out.println("\n\n\n hit 'p,Enter' for pause, 'x,Enter' to stop and terminate. \n\n\n");

		// "F:/AGH/LabRobotow/CapoRobot/LokalizacjaJava/CapoMazeHough/res/MazeRoboLabFullMap2.roson"
		CapoFearBasedControllerMock capoController;
		try
		{
			capoController = new CapoFearBasedControllerMock(Integer.parseInt(args[0]), Double.parseDouble(args[1]), args[2], Double.parseDouble(args[7]), Double.parseDouble(args[8]), Double.parseDouble(args[9]), Integer.parseInt(args[10]));
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

		while (true)
		{
			int c = System.in.read();
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
