package pl.edu.agh.capo.fear;

public class SensorLoopMonitorThread implements Runnable
{
	public static final double maxSensorRefreshTimeSeconds = 0.3D;
	protected boolean stop = false;
	protected CapoFearBasedController capoController;
	protected CapoFearBasedControllerMock capoControllerMock;

	public void Stop()
	{
		this.stop = true;
	}

	public SensorLoopMonitorThread(CapoFearBasedController capoController)
	{
		this.capoController = capoController;
		this.capoControllerMock = null;
	}

	public SensorLoopMonitorThread(CapoFearBasedControllerMock capoControllerMock)
	{
		this.capoControllerMock = capoControllerMock;
		this.capoController = null;
	}

	public void run()
	{
		while (!this.stop)
		{
			Thread.interrupted();
			try
			{
				Thread.sleep(300L);
			}
			catch (InterruptedException e)
			{
				continue;
			}

			if (capoController != null)
				this.capoController.reduceSpeedDueToSensorRedingTimeout();
			else
				this.capoControllerMock.reduceSpeedDueToSensorRedingTimeout();
		}
	}
}
