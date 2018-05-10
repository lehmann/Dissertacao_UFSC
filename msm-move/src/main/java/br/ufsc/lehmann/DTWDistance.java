package br.ufsc.lehmann;

import br.ufsc.core.trajectory.IDistanceFunction;
import br.ufsc.core.trajectory.SpatialDistanceFunction;
import br.ufsc.core.trajectory.TPoint;
import smile.math.distance.DynamicTimeWarping;

public class DTWDistance implements IDistanceFunction<TPoint[]> {

	private DynamicTimeWarping<TPoint> dtw;

	public DTWDistance(SpatialDistanceFunction func) {
		dtw = new DynamicTimeWarping<TPoint>(new SmileDistanceWrapper(func));
	}

	public DTWDistance(SpatialDistanceFunction distanceFunction, Number i) {
		this(distanceFunction);
	}

	@Override
	public double distance(TPoint[] p, TPoint[] d) {
		if(p == d) {
			return 0;
		}
		if (p == null || d == null) {
			return 1;
		}
		return dtw.d(p, d);
	}

	@Override
	public double convert(double units) {
		return units;
	}
	
	@Override
	public double maxDistance() {
		return Double.POSITIVE_INFINITY;
	}

}
