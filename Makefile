VENV   := .venv
PYTHON := $(VENV)/bin/python

.PHONY: help data clean-data

help:
	@echo "make data        - download maps and convert them to C++ graphs (data/*.gr)"
	@echo "make clean-data  - delete data/ and OSMnx cache (cache/)"

# Create .venv and install deps; re-run when requirements.txt changes
$(VENV)/.installed: requirements.txt
	python3 -m venv $(VENV)
	$(PYTHON) -m pip install -r requirements.txt
	touch $@

data: $(VENV)/.installed
	$(PYTHON) scripts/download_maps.py
	$(PYTHON) scripts/convert_maps.py

clean-data:
	rm -rf data/ cache/
