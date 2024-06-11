# Copyright 2024 Zivid AS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Zivid AS nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


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
