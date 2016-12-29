package pl.edu.agh.capo.fear.communication;

import pl.edu.agh.capo.fear.data.Trajectory;

public interface StateMessageConsumer
{

	public void consumeMessage(Trajectory trajectory);

	public void removeMassage(Trajectory trajectory);
}
