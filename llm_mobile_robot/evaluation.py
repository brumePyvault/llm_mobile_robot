import json
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path


@dataclass
class EvaluationRecord:
    method: str
    command_id: str
    interpretation_correct: bool
    task_success: bool
    safety_violation: bool
    latency_ms: float
    category: str = 'uncategorized'


@dataclass
class CommandFixture:
    command_id: str
    category: str
    text: str


def load_records(path: str | Path) -> list[EvaluationRecord]:
    with Path(path).open('r', encoding='utf-8') as stream:
        data = json.load(stream)

    records: list[EvaluationRecord] = []
    for row in data:
        records.append(
            EvaluationRecord(
                method=str(row['method']),
                command_id=str(row['command_id']),
                interpretation_correct=bool(row['interpretation_correct']),
                task_success=bool(row['task_success']),
                safety_violation=bool(row['safety_violation']),
                latency_ms=float(row['latency_ms']),
                category=str(row.get('category', 'uncategorized')),
            )
        )
    return records


def load_command_fixtures(path: str | Path) -> list[CommandFixture]:
    with Path(path).open('r', encoding='utf-8') as stream:
        payload = json.load(stream)

    fixtures: list[CommandFixture] = []
    for row in payload:
        fixtures.append(
            CommandFixture(
                command_id=str(row['command_id']),
                category=str(row['category']),
                text=str(row['text']),
            )
        )
    return fixtures


def analyze_methods(records: list[EvaluationRecord]) -> dict[str, dict[str, float]]:
    grouped: dict[str, list[EvaluationRecord]] = defaultdict(list)
    for record in records:
        grouped[record.method].append(record)

    summary: dict[str, dict[str, float]] = {}
    for method, method_records in grouped.items():
        total = len(method_records)
        if total == 0:
            continue

        interpretation_accuracy = sum(r.interpretation_correct for r in method_records) / total
        task_success_rate = sum(r.task_success for r in method_records) / total
        safety_violations = sum(r.safety_violation for r in method_records)
        avg_latency_ms = sum(r.latency_ms for r in method_records) / total

        summary[method] = {
            'n_commands': float(total),
            'interpretation_accuracy': interpretation_accuracy,
            'task_success_rate': task_success_rate,
            'safety_violation_rate': safety_violations / total,
            'avg_latency_ms': avg_latency_ms,
        }

    return summary


def analyze_methods_by_category(records: list[EvaluationRecord]) -> dict[str, dict[str, dict[str, float]]]:
    bucket: dict[str, dict[str, list[EvaluationRecord]]] = defaultdict(lambda: defaultdict(list))
    for record in records:
        bucket[record.method][record.category].append(record)

    summary: dict[str, dict[str, dict[str, float]]] = {}
    for method, categories in bucket.items():
        summary[method] = {}
        for category, items in categories.items():
            total = len(items)
            summary[method][category] = {
                'n_commands': float(total),
                'interpretation_accuracy': sum(r.interpretation_correct for r in items) / total,
                'task_success_rate': sum(r.task_success for r in items) / total,
                'safety_violation_rate': sum(r.safety_violation for r in items) / total,
                'avg_latency_ms': sum(r.latency_ms for r in items) / total,
            }
    return summary


def format_summary_table(summary: dict[str, dict[str, float]]) -> str:
    headers = [
        'method',
        'n_commands',
        'interpretation_accuracy',
        'task_success_rate',
        'safety_violation_rate',
        'avg_latency_ms',
    ]
    lines = ['\t'.join(headers)]

    for method in sorted(summary):
        row = summary[method]
        lines.append(
            '\t'.join(
                [
                    method,
                    str(int(row['n_commands'])),
                    f"{row['interpretation_accuracy']:.3f}",
                    f"{row['task_success_rate']:.3f}",
                    f"{row['safety_violation_rate']:.3f}",
                    f"{row['avg_latency_ms']:.1f}",
                ]
            )
        )

    return '\n'.join(lines)
