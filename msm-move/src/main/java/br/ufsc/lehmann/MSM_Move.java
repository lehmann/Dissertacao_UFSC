package br.ufsc.lehmann;

import br.ufsc.core.trajectory.Semantic;
import br.ufsc.core.trajectory.SemanticTrajectory;
import br.ufsc.core.trajectory.semantic.Move;
import br.ufsc.core.trajectory.semantic.Stop;
import br.ufsc.ftsm.base.TrajectorySimilarityCalculator;
import br.ufsc.ftsm.related.MSM;
import br.ufsc.ftsm.related.MSM.MSMSemanticParameter;

public class MSM_Move extends TrajectorySimilarityCalculator<SemanticTrajectory> {
	
	private MSM msm;
	private double moveWeight;
	private MSMMoveSemanticParameter moveParams;
	
	public MSM_Move(MSMMoveSemanticParameter moveParams, MSMSemanticParameter<?, ?>... params) {
		if(moveParams == null) {
			throw new IllegalArgumentException("Params can not be null");
		}
		this.moveParams = moveParams;
		this.moveWeight = moveParams.getWeight();
		if(params != null) {
			this.msm = new MSM(params);
		}
	}

	@Override
	public double getSimilarity(SemanticTrajectory t1, SemanticTrajectory t2) {
		return (msm != null ? msm.getSimilarity(t1, t2) : 0) * (1 - moveWeight) + moveSimilarity(t1, t2) * moveWeight;
	}

	public double moveSimilarity(SemanticTrajectory A, SemanticTrajectory B) {
		int n = A.length();
		int m = B.length();
		double bScore[] = new double[m];
		
		double parityAB = 0.0;
		
		for (int i = 0; i < n; i++) {
			double maxScore = 0.0;
			Move moveA = moveParams.getMoveSemantic().getData(A, i);
			if(moveA == null) {
				continue;
			}
		
			for (int j = 0; j < m; j++) {
				double score = 0.0;
				double semanticScore = 0;
				Move moveB = moveParams.getMoveSemantic().getData(B, j);
				if(moveB == null) {
					continue;
				}
				semanticScore += (moveParams.getStopSemantic().match(moveA.getStart(), moveB.getStart(), moveParams.getStopThreshold()) ? 1 : 0);
				semanticScore += (moveParams.getStopSemantic().match(moveA.getEnd(), moveB.getEnd(), moveParams.getStopThreshold()) ? 1 : 0);
				semanticScore += (moveParams.getMoveSemantic().match(moveA, moveB, moveParams.getMoveThreshold()) ? 1 : 0);
				score = semanticScore / 3;
		
				if (score >= maxScore) {
					maxScore = score;
					bScore[j] = maxScore > bScore[j] ? maxScore : bScore[j];
				}
			}
			parityAB += maxScore;
		}
		
		double parityBA = 0;
		for (int j = 0; j < m; j++) {
			parityBA += bScore[j];
		}
		
		double similarity = (parityAB + parityBA) / (n + m);

		return similarity;
	}
	
	public static class MSMMoveSemanticParameter {
		private double weight;
		private Semantic<Stop, Number> stopSemantic;
		private Semantic<Move, Number> moveSemantic;
		private Number stopThreshold;
		private Number moveThreshold;
		public MSMMoveSemanticParameter(Semantic<Stop, Number> stopSemantic, Number stopThreshold, Semantic<Move, Number> moveSemantic, Number moveThreshold, double weight) {
			super();
			this.stopSemantic = stopSemantic;
			this.stopThreshold = stopThreshold;
			this.moveSemantic = moveSemantic;
			this.moveThreshold = moveThreshold;
			this.weight = weight;
		}
		public double getWeight() {
			return weight;
		}
		public Semantic<Stop, Number> getStopSemantic() {
			return stopSemantic;
		}
		public Semantic<Move, Number> getMoveSemantic() {
			return moveSemantic;
		}
		public Number getStopThreshold() {
			return stopThreshold;
		}
		public Number getMoveThreshold() {
			return moveThreshold;
		}
	}
}
