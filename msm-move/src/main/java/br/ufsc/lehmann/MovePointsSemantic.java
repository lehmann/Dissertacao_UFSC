package br.ufsc.lehmann;

import br.ufsc.core.trajectory.GeographicDistanceFunction;
import br.ufsc.core.trajectory.Semantic;
import br.ufsc.core.trajectory.SemanticTrajectory;
import br.ufsc.core.trajectory.TPoint;
import br.ufsc.core.trajectory.semantic.Move;
import smile.math.distance.DynamicTimeWarping;

public class MovePointsSemantic extends Semantic<Move, Number> {

	private DynamicTimeWarping<TPoint> dtw;

	public MovePointsSemantic(int index, GeographicDistanceFunction func, Number geographicThreshold) {
		super(index);
		dtw = new DynamicTimeWarping<TPoint>(new SmileDistanceWrapper(func, geographicThreshold));
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
		TPoint[] d2Points = d2.getPoints();
		TPoint[] d1Points = d1.getPoints();
		return dtw.d(d1Points, d2Points) / Math.max(d1Points.length, d2Points.length);
	}

}
