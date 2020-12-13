.PHONY: setup
setup:
	pip install catkin_pkg
	pip install -r requirements.txt
	pip install -e .

.PHONY: test
test:
	python -m pytest test

.PHONY: circleci-local
circleci-local:
	circleci local execute --job test
