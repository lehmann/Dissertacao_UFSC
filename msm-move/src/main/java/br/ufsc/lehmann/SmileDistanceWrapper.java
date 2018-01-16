package br.ufsc.lehmann;

import br.ufsc.core.trajectory.SpatialDistanceFunction;
import br.ufsc.core.trajectory.TPoint;
import smile.math.distance.Distance;

public class SmileDistanceWrapper implements Distance<TPoint> {

	private SpatialDistanceFunction measure;

	public SmileDistanceWrapper(SpatialDistanceFunction measure) {
		this.measure = measure;
	}

	@Override
	public double d(TPoint x, TPoint y) {
		return measure.distance(x, y);
	}
	
}