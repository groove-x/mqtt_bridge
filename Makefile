.PHONY: setup
setup:
	pip install catkin_pkg
	pip install -r dev-requirements.txt
	pip install -e .

.PHONY: test
test:
	python -m pytest test

.PHONY: circleci-local
circleci-local:
	circleci local execute --job test
	circleci local execute --job rostest
