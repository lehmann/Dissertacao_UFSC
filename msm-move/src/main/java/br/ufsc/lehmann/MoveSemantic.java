package br.ufsc.lehmann;

import br.ufsc.core.trajectory.Semantic;
import br.ufsc.core.trajectory.SemanticTrajectory;
import br.ufsc.core.trajectory.semantic.Move;

public class MoveSemantic extends Semantic<Move, Number> {

	private Fields field;

	public MoveSemantic(int index, Fields field) {
		super(index);
		this.field = field;
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
		if(field == Fields.ANGLE) {
			double phi = Math.abs(d2.getAngle() - d1.getAngle()) % 360;
			double distance = phi > 180 ? 360 - phi : phi;
			return (distance);
		} else if(field == Fields.DISTANCE) {
			double distance = Math.abs(d2.getTraveledDistance() - d1.getTraveledDistance());
			return distance;
		}
		throw new IllegalStateException("Unknown move field estimator: " + field.name());
	}

	public static enum Fields {
		ANGLE, DISTANCE;
	}
}
