Always run the following for testing:

```bash
pylint . # ensure a score of 9 or higher
```
```bash
flake8 main.py test.py # must have no errors, including too complex
```
```bash
pytest -v -n 12
```



