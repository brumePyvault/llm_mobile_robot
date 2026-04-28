from llm_mobile_robot.evaluation import EvaluationRecord, analyze_methods, analyze_methods_by_category


def test_analyze_methods_three_strategies():
    records = [
        EvaluationRecord('zero_shot', 'c1', True, True, False, 510, 'A_single_step'),
        EvaluationRecord('zero_shot', 'c2', False, False, True, 650, 'F_safety_invalid'),
        EvaluationRecord('few_shot', 'c1', True, True, False, 590, 'A_single_step'),
        EvaluationRecord('few_shot', 'c2', True, True, False, 610, 'B_multi_step'),
        EvaluationRecord('fine_tuned', 'c1', True, True, False, 480, 'C_reasoning'),
        EvaluationRecord('fine_tuned', 'c2', True, False, False, 495, 'D_conditional'),
    ]

    summary = analyze_methods(records)

    assert summary['zero_shot']['interpretation_accuracy'] == 0.5
    assert summary['few_shot']['task_success_rate'] == 1.0
    assert summary['fine_tuned']['avg_latency_ms'] == 487.5


def test_analyze_methods_by_category():
    records = [
        EvaluationRecord('few_shot', 'c1', True, True, False, 590, 'A_single_step'),
        EvaluationRecord('few_shot', 'c2', True, False, False, 610, 'A_single_step'),
    ]

    summary = analyze_methods_by_category(records)

    assert summary['few_shot']['A_single_step']['n_commands'] == 2.0
    assert summary['few_shot']['A_single_step']['task_success_rate'] == 0.5
