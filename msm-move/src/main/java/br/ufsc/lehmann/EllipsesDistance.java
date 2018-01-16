package br.ufsc.lehmann;

import java.util.Objects;

import br.ufsc.core.trajectory.IDistanceFunction;
import br.ufsc.core.trajectory.SpatialDistanceFunction;
import br.ufsc.core.trajectory.TPoint;
import br.ufsc.ftsm.base.ETrajectory;
import br.ufsc.ftsm.related.UMS;
import br.ufsc.ftsm.util.CreateEllipseMath;
import br.ufsc.utils.EuclideanDistanceFunction;

public class EllipsesDistance implements IDistanceFunction<TPoint[]> {

	private UMS ums;
	private SpatialDistanceFunction distanceFunction;

	public EllipsesDistance() {
		this(new EuclideanDistanceFunction());
	}

	public EllipsesDistance(SpatialDistanceFunction distanceFunction) {
		this.distanceFunction = distanceFunction;
		this.ums = new UMS(distanceFunction);
	}

	@Override
	public double distance(TPoint[] p, TPoint[] d) {
		if (p == null || d == null) {
			return 1;
		}
		if(Objects.deepEquals(p, d)) {
			return 0;
		}
		TPoint[] mercatorP = distanceFunction.convertToMercator(p);
		TPoint[] mercatorD = distanceFunction.convertToMercator(d);
		CreateEllipseMath ellipseMath = new CreateEllipseMath();
		ETrajectory T1 = ellipseMath.createEllipticalTrajectoryFixed(-1, mercatorP);
		ETrajectory T2 = ellipseMath.createEllipticalTrajectoryFixed(1, mercatorD);
		double distance = 1 - ums.getSimilarity(//
				T1, //
				T2);
		return distance;
	}

	@Override
	public double convert(double units) {
		return units;
	}
	
	@Override
	public double maxDistance() {
		return 1;
	}

}
