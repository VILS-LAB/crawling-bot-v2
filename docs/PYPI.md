# Publishing the Python client to PyPI

The Python client is published from [`examples/python`](../examples/python) as the
distribution **`crawlingbot`**. The build metadata is plain
[`examples/python/pyproject.toml`](../examples/python/pyproject.toml) and the
version has one source of truth: `__version__` in
[`crawlingbot/__init__.py`](../examples/python/crawlingbot/__init__.py).

Students install it with:

```bash
pip install crawlingbot            # TCP, no third-party dependencies
pip install "crawlingbot[serial]"  # adds pyserial for Robot.serial(...)
```

## One-time setup

1. **Create the PyPI accounts.** Sign up on both <https://pypi.org> and
   <https://test.pypi.org> and verify your email. Enable two-factor auth — PyPI
   requires it for uploads.
2. **Reserve the name.** `crawlingbot` was unclaimed at the time of writing.
   Names are claimed by the first successful upload, so publish a `0.1.0` or
   `1.0.0` to TestPyPI early rather than waiting.
3. **Set up trusted publishing (no API tokens to leak).** On PyPI open
   *Your projects → Publishing* (or *Account → Publishing* before the project
   exists) and add a GitHub publisher with:
   - owner: `VILS-LAB`
   - repository: `crawling-bot-v2`
   - workflow: `publish-python-client.yml`
   - environment: `pypi` (and a second entry with `testpypi` for TestPyPI)

   The owner must be the organisation that owns the repository today, not a
   former personal account: PyPI compares these fields against the OIDC claims
   GitHub sends at upload time, and a mismatch fails with `403`.

   Then create the matching environments in GitHub under
   *Settings → Environments*: `pypi` and `testpypi` (add required reviewers on
   `pypi` if you want a manual gate before each real release).
4. **Keep the license.** The build root needs its own copy of the license file
   because setuptools/hatchling cannot reference a file outside the project
   directory. `examples/python/LICENSE` is a copy of the repository root
   `LICENSE` — update both together.

## Local dry run

Everything below is safe and installs nothing into the repository:

```bash
cd examples/python
python -m pip install --upgrade build "twine>=7" pytest

python -m pytest -q                 # 20 tests, no hardware needed
python -m build                     # writes dist/*.whl and dist/*.tar.gz
python -m twine check --strict dist/*
```

Inspect what will actually ship:

```bash
python -m zipfile -l dist/crawlingbot-*.whl
```

Then confirm the wheel works on its own, away from the source tree. Note the
`cd /tmp` and `-I`: run from `examples/python` and Python would import the
checkout instead of the installed wheel, so the check would pass even for a
broken artifact.

```bash
python -m venv /tmp/smoke
/tmp/smoke/bin/pip install dist/crawlingbot-*.whl
cd /tmp
/tmp/smoke/bin/python -I -c "import crawlingbot; print(crawlingbot.__file__)"
```

The printed path must be inside `site-packages`.

## Manual upload (first release, or when you are not using CI)

```bash
# TestPyPI first
python -m twine upload --repository testpypi dist/*

# install exactly what you just uploaded
python -m pip install --index-url https://test.pypi.org/simple/ crawlingbot

# then the real thing
python -m twine upload dist/*
```

`twine` asks for a username and password. Use `__token__` as the username and a
[project-scoped API token](https://pypi.org/manage/account/token/) as the
password. Do not paste tokens into files in this repository.

## Releases through GitHub Actions

[`publish-python-client.yml`](../.github/workflows/publish-python-client.yml)
builds, tests, checks, smoke-tests and uploads the artifacts.

* **TestPyPI:** *Actions → Publish Python client to PyPI → Run workflow*,
  target `testpypi`.
* **PyPI:** publish a GitHub Release whose tag starts with `python-v` (or run
  the workflow with target `pypi`). Both PyPI jobs use OIDC trusted publishing,
  so no secrets are stored.

The tag gate matters: this repository's releases are usually firmware releases,
and those must not upload a Python package. A firmware release simply skips the
PyPI job. The build job also refuses a `python-v*` release whose tag does not
match `__version__`, so a typo cannot publish the wrong version.

Because TestPyPI and PyPI are separate indexes, a failed upload to one never
blocks the other.

## Cutting a version

1. Bump `__version__` in
   [`crawlingbot/__init__.py`](../examples/python/crawlingbot/__init__.py).
2. Add a matching entry to [`CHANGELOG.md`](../CHANGELOG.md).
3. Run the local dry run above.
4. Commit, then publish the GitHub Release. The tag **must** be
   `python-v<version>` (for example `python-v1.1.0`) so it stays distinct from
   firmware tags and matches `__version__`; the workflow verifies both.

### Version numbers are permanent

PyPI rejects a second upload of the same version, even if the first one was
deleted or is broken. If a release is wrong, bump the patch number and publish
again; `1.0.0` can never be reused.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `File already exists` on upload | That version is already on the index. Bump `__version__`. |
| `Invalid or non-existent authentication information` | Use `__token__` as the username and the full `pypi-...` token as the password. |
| `403` from trusted publishing | The PyPI publisher entry's owner/repository/workflow/environment must match the GitHub workflow exactly. |
| `InvalidDistribution: ... '2.5' is not a valid metadata version` | Your `twine` is older than 7 and cannot read the metadata hatchling emits. Install `twine>=7`. |
| `RuntimeError: OutsideDestinationError` while building | A `license-files`/`force-include` path points outside the project. Keep license copies inside `examples/python`. |
| Students on Python 3.7 get an error | The package requires Python 3.8+ (`requires-python`), which pip enforces. |
