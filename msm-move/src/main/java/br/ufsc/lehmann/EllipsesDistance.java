package br.ufsc.lehmann;

import br.ufsc.core.trajectory.GeographicDistanceFunction;
import br.ufsc.core.trajectory.IDistanceFunction;
import br.ufsc.core.trajectory.TPoint;
import br.ufsc.ftsm.base.ETrajectory;
import br.ufsc.ftsm.related.UMS;
import br.ufsc.ftsm.util.CreateEllipseMath;

public class EllipsesDistance implements IDistanceFunction<TPoint[]> {

	private UMS ums;
	private GeographicDistanceFunction distanceFunction;

	public EllipsesDistance(GeographicDistanceFunction distanceFunction) {
		this.distanceFunction = distanceFunction;
		this.ums = new UMS();
	}

	@Override
	public double distance(TPoint[] p, TPoint[] d) {
		if (p == null || d == null) {
			return 1;
		}
		TPoint[] mercatorP = distanceFunction.convertToMercator(p);
		TPoint[] mercatorD = distanceFunction.convertToMercator(d);
		CreateEllipseMath ellipseMath = new CreateEllipseMath();
		ETrajectory T1 = ellipseMath.createEllipticalTrajectoryFixed(-1, mercatorP);
		ETrajectory T2 = ellipseMath.createEllipticalTrajectoryFixed(1, mercatorD);
		return 1 - ums.getSimilarity(//
				T1, //
				T2);
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
