import argparse
import os
import requests


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--status", required=True, help="Job status", type=str)
    return parser.parse_args()


def _main() -> None:
    args = _args()

    job = os.environ["GITHUB_JOB"]
    branch = "/".join(os.environ["GITHUB_REF"].split("/")[2:])
    workflow = os.environ["GITHUB_WORKFLOW"]
    run_sha = os.environ["GITHUB_SHA"]
    run_number = os.environ["GITHUB_RUN_NUMBER"]

    server_url = os.environ["GITHUB_SERVER_URL"]
    repo = os.environ["GITHUB_REPOSITORY"]
    run_id = os.environ["GITHUB_RUN_ID"]
    workflow_url = f"{server_url}/{repo}/actions/runs/{run_id}"
    webhook_url = os.environ["CI_FAILURE_TEAMS_HOOK"]

    payload = {
        "@type": "MessageCard",
        "@context": "http://schema.org/extensions",
        "themeColor": "EC00EC",
        "summary": f"{job} {args.status} on {branch}",
        "sections": [
            {
                "activityTitle": f"{branch} {args.status}!",
                "activitySubtitle": f"In workflow '{workflow}', job '{job}'",
                "facts": [
                    {"name": "Sha", "value": run_sha},
                    {"name": "Run", "value": run_number},
                ],
            }
        ],
        "potentialAction": [
            {
                "@type": "ActionCard",
                "name": "Actions",
                "actions": [
                    {
                        "@type": "OpenUri",
                        "name": "Go to GitHub Actions",
                        "targets": [{"os": "default", "uri": workflow_url}],
                    }
                ],
            }
        ],
    }
    request = requests.post(webhook_url, json=payload)
    try:
        request.raise_for_status()
    except requests.HTTPError as ex:
        raise RuntimeError("Failed to post Teams notification") from ex


if __name__ == "__main__":
    _main()
