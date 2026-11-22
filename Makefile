VENV   := .venv
PYTHON := $(VENV)/bin/python

.PHONY: help maps clean-maps

help:
	@echo "make maps        - download Hsinchu + Taiwan maps"
	@echo "make clean-maps  - delete downloaded maps (data/) and OSMnx cache (cache/)"

# Create .venv and install deps; re-run when requirements.txt changes
$(VENV)/.installed: requirements.txt
	python3 -m venv $(VENV)
	$(PYTHON) -m pip install -r requirements.txt
	touch $@

maps: $(VENV)/.installed
	$(PYTHON) scripts/download_maps.py

clean-maps:
	rm -rf data/ cache/
