package br.ufsc.lehmann;

import java.util.ArrayList;
import java.util.List;

import br.ufsc.core.trajectory.Semantic;
import br.ufsc.core.trajectory.SemanticTrajectory;
import br.ufsc.core.trajectory.semantic.Move;
import br.ufsc.core.trajectory.semantic.Stop;
import br.ufsc.ftsm.base.TrajectorySimilarityCalculator;
import br.ufsc.ftsm.related.MSM;
import br.ufsc.ftsm.related.MSM.MSMSemanticParameter;

public class SMSM extends TrajectorySimilarityCalculator<SemanticTrajectory> {
	
	private MSM msm;
	private double moveWeight;
	private SMSMSemanticParameter moveParams;
	
	public SMSM(SMSMSemanticParameter moveParams, MSMSemanticParameter<?, ?>... params) {
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
		List<Move> tuplesA = tuples(A);
		List<Move> tuplesB = tuples(B);
		int n = tuplesA.size();
		int m = tuplesB.size();
		double bScore[] = new double[m];
		
		double parityAB = 0.0;
		
		for (int i = 0; i < n; i++) {
			double maxScore = 0.0;
			Move moveA = tuplesA.get(i);
		
			for (int j = 0; j < m; j++) {
				double score = 0.0;
				double semanticScore = 0;
				Move moveB = tuplesB.get(j);
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
	
	private List<Move> tuples(SemanticTrajectory a) {
		List<Move> ret = new ArrayList<Move>();
		for (int i = 0; i < a.length(); i++) {
			Move move = moveParams.getMoveSemantic().getData(a, i);
			if(move != null) {
				ret.add(move);
			}
		}
		return ret;
	}

	public static class SMSMSemanticParameter {
		private double weight;
		private Semantic<Stop, Number> stopSemantic;
		private Semantic<Move, Number> moveSemantic;
		private Number stopThreshold;
		private Number moveThreshold;
		public SMSMSemanticParameter(Semantic<Stop, Number> stopSemantic, Number stopThreshold, Semantic<Move, Number> moveSemantic, Number moveThreshold, double weight) {
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
