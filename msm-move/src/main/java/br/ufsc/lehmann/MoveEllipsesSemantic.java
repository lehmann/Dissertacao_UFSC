package br.ufsc.lehmann;

import br.ufsc.core.trajectory.Semantic;
import br.ufsc.core.trajectory.SemanticTrajectory;
import br.ufsc.core.trajectory.TPoint;
import br.ufsc.core.trajectory.semantic.Move;
import br.ufsc.ftsm.base.ETrajectory;
import br.ufsc.ftsm.related.UMS;
import br.ufsc.ftsm.util.CreateEllipseMath;

public class MoveEllipsesSemantic extends Semantic<Move, Number> {

	private UMS ums;

	public MoveEllipsesSemantic(int index) {
		super(index);
		ums = new UMS();
	}

	@Override
	public Number distance(SemanticTrajectory a, int i, SemanticTrajectory b, int j) {
		return distance(getData(a, i), getData(b, j));
	}

	@Override
	public boolean match(SemanticTrajectory a, int i, SemanticTrajectory b, int j, Number threshold) {
		return match(getData(a, i), getData(b, j), threshold);
	}

	@Override
	public boolean match(Move d1, Move d2, Number threshold) {
		return distance(d1, d2) <= (threshold == null ? 0 : threshold.doubleValue());
	}

	@Override
	public double distance(Move d1, Move d2) {
		if(d1 == d2) {
			return 0;
		}
		if (d1 == null || d2 == null) {
			return Double.MAX_VALUE;
		}
		TPoint[] d1Points = d1.getPoints();
		TPoint[] d2Points = d2.getPoints();
		ETrajectory T1 = CreateEllipseMath.createEllipticalTrajectoryFixed(d1.getMoveId(), d1Points);
		ETrajectory T2 = CreateEllipseMath.createEllipticalTrajectoryFixed(d2.getMoveId(), d2Points);
		return ums.getDistance(//
				T1, //
				T2);
	}

}
