package br.ufsc.lehmann;

import br.ufsc.core.trajectory.SpatialDistanceFunction;
import br.ufsc.core.trajectory.TPoint;
import smile.math.distance.Distance;

public class SmileDistanceWrapper implements Distance<TPoint> {

	private SpatialDistanceFunction measure;
	private Number threshold;

	public SmileDistanceWrapper(SpatialDistanceFunction measure, Number threshold) {
		this.measure = measure;
		this.threshold = threshold;
	}

	@Override
	public double d(TPoint x, TPoint y) {
		return measure.distance(x, y) > threshold.doubleValue() ? 1 : 0;
	}
	
}