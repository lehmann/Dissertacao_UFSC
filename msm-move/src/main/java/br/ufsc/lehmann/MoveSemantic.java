package br.ufsc.lehmann;

import java.util.Objects;

import br.ufsc.core.trajectory.Semantic;
import br.ufsc.core.trajectory.SemanticTrajectory;
import br.ufsc.core.trajectory.semantic.Move;

public class MoveSemantic extends Semantic<Move, Number> {

	public MoveSemantic(int index) {
		super(index);
	}

	@Override
	public Number distance(SemanticTrajectory a, int i, SemanticTrajectory b, int j) {
		return distance(getData(a, i), getData(b, j));
	}

	@Override
	public boolean match(SemanticTrajectory a, int i, SemanticTrajectory b, int j, Number threshold) {
		return distance(a, i, b, j).doubleValue() <= (threshold == null ? 0 : threshold.doubleValue());
	}

	@Override
	public double distance(Move d1, Move d2) {
		if(d1 == d2) {
			return 0;
		}
		if (d1 == null || d2 == null) {
			return 1;
		}
		double score = ((Objects.equals(d1.getStart(), d2.getStart()) ? 1.0 : 0.0)
				+ (Objects.equals(d1.getEnd(), d2.getEnd()) ? 1.0 : 0.0)
				+ (d1.getMoveId() == d2.getMoveId() ? 1.0 : 0.0))
			/ 3.0;
		return 1 - score;
	}

}
