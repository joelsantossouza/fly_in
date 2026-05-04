MAIN = main.py

install:
	python3 -m pip install --upgrade pip
	python3 -m pip install -r requirements.txt

run:
	python3 $(MAIN) ./maps/challenger/01_the_impossible_dream.txt

debug:
	python3 -m pdb $(MAIN)

clean:
	find . -type d -name "__pycache__" -exec rm -rf {} +
	rm -rf .mypy_cache
	rm -rf .pytest_cache

lint:
	flake8 .
	mypy . --warn-return-any --warn-unused-ignores \
           --ignore-missing-imports --disallow-untyped-defs \
           --check-untyped-defs

lint-strict:
	flake8 .
	mypy . --strict

