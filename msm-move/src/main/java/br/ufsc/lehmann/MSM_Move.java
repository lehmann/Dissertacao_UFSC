package br.ufsc.lehmann;

import br.ufsc.core.trajectory.SemanticTrajectory;
import br.ufsc.core.trajectory.semantic.Move;
import br.ufsc.ftsm.base.TrajectorySimilarityCalculator;
import br.ufsc.ftsm.related.MSM;
import br.ufsc.ftsm.related.MSM.MSMSemanticParameter;

public class MSM_Move extends TrajectorySimilarityCalculator<SemanticTrajectory> {
	
	private MSM moveMsm;
	private MSM msm;
	private double moveWeight;
	
	public MSM_Move(MSMSemanticParameter<Move, Number> moveParams, MSMSemanticParameter<?, ?>... params) {
		if(params == null) {
			throw new IllegalArgumentException("Params can not be null");
		}
		MSMSemanticParameter<Move, Number> moveSemantic = moveParams;
		this.moveMsm = new MSM(new MSMSemanticParameter<Move, Number>(moveSemantic.getSemantic(), moveSemantic.getThreshlod(), 1));
		this.moveWeight = moveSemantic.getWeight();
		this.msm = new MSM(params);
	}

	@Override
	public double getSimilarity(SemanticTrajectory t1, SemanticTrajectory t2) {
		return msm.getSimilarity(t1, t2) * (1 - moveWeight) + moveMsm.getSimilarity(t1, t2) * moveWeight;
	}

	
}
