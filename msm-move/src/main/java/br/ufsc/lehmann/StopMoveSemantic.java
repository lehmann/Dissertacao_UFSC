package br.ufsc.lehmann;

import br.ufsc.core.trajectory.SemanticTrajectory;
import br.ufsc.core.trajectory.StopSemantic;
import br.ufsc.core.trajectory.semantic.Move;

public class StopMoveSemantic extends MoveSemantic {

	private StopSemantic stopSemantic;

	public StopMoveSemantic(int index, StopSemantic stopSemantic) {
		super(index);
		this.stopSemantic = stopSemantic;
	}
	
	@Override
	public double distance(Move d1, Move d2) {
		if(d1 == d2) {
			return 0;
		}
		if (d1 == null || d2 == null) {
			return 1;
		}
		double phi = Math.abs(d2.getAngle() - d1.getAngle()) % 360;
		double angleDistance = (phi > 180 ? 360 - phi : phi) / 180;
		double distance = (stopSemantic.distance(d1.getStart(), d2.getStart())
				+ (stopSemantic.distance(d1.getEnd(), d2.getEnd()))
				+ (angleDistance))
			/ 3.0;
		return distance;
	}
	
	@Override
	public boolean match(SemanticTrajectory a, int i, SemanticTrajectory b, int j, Number threshold) {
		return super.match(a, i, b, j, threshold);
	}

}
