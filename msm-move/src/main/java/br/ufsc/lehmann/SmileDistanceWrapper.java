package br.ufsc.lehmann;

import br.ufsc.core.trajectory.GeographicDistanceFunction;
import br.ufsc.core.trajectory.TPoint;
import smile.math.distance.Distance;

public class SmileDistanceWrapper implements Distance<TPoint> {

	private GeographicDistanceFunction measure;
	private Number threshold;

	public SmileDistanceWrapper(GeographicDistanceFunction measure, Number threshold) {
		this.measure = measure;
		this.threshold = threshold;
	}

	@Override
	public double d(TPoint x, TPoint y) {
		return measure.distance(x, y) > threshold.doubleValue() ? 1 : 0;
	}
	
}