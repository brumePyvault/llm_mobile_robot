from llm_mobile_robot.evaluation import EvaluationRecord, analyze_methods


def test_analyze_methods_three_strategies():
    records = [
        EvaluationRecord('zero_shot', 'c1', True, True, False, 510),
        EvaluationRecord('zero_shot', 'c2', False, False, True, 650),
        EvaluationRecord('few_shot', 'c1', True, True, False, 590),
        EvaluationRecord('few_shot', 'c2', True, True, False, 610),
        EvaluationRecord('fine_tuned', 'c1', True, True, False, 480),
        EvaluationRecord('fine_tuned', 'c2', True, False, False, 495),
    ]

    summary = analyze_methods(records)

    assert summary['zero_shot']['interpretation_accuracy'] == 0.5
    assert summary['few_shot']['task_success_rate'] == 1.0
    assert summary['fine_tuned']['avg_latency_ms'] == 487.5
