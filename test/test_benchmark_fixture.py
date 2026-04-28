from llm_mobile_robot.evaluation import load_command_fixtures


def test_benchmark_fixture_contains_expected_categories():
    fixtures = load_command_fixtures('test/fixtures/benchmark_commands.json')

    categories = {item.category for item in fixtures}
    assert len(fixtures) == 200
    assert 'A_single_step' in categories
    assert 'F_safety_invalid' in categories
