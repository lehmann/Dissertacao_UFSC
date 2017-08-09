package br.ufsc.lehmann;

import br.ufsc.core.trajectory.IDistanceFunction;
import br.ufsc.core.trajectory.TPoint;
import br.ufsc.ftsm.base.ETrajectory;
import br.ufsc.ftsm.related.UMS;
import br.ufsc.ftsm.util.CreateEllipseMath;

public class EllipsesDistance implements IDistanceFunction<TPoint[]> {

	private UMS ums;

	public EllipsesDistance() {
		this.ums = new UMS();
	}

	@Override
	public double distance(TPoint[] p, TPoint[] d) {
		if(p == d) {
			return 0;
		}
		if (p == null || d == null) {
			return Double.MAX_VALUE;
		}
		ETrajectory T1 = CreateEllipseMath.createEllipticalTrajectoryFixed(-1, p);
		ETrajectory T2 = CreateEllipseMath.createEllipticalTrajectoryFixed(1, d);
		return ums.getDistance(//
				T1, //
				T2);
	}

	@Override
	public double convert(double units) {
		return units;
	}

}
