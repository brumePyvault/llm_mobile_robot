#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from llm_mobile_robot.evaluation import analyze_methods, format_summary_table, load_records


def main() -> None:
    parser = argparse.ArgumentParser(description='Analyze zero-shot/few-shot/fine-tuned run logs.')
    parser.add_argument('results_json', help='Path to JSON file containing evaluation records.')
    args = parser.parse_args()

    records = load_records(args.results_json)
    summary = analyze_methods(records)
    print(format_summary_table(summary))


if __name__ == '__main__':
    main()
