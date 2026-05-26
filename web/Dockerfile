FROM python:3.11-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    LAP_ESTIMATOR_REPO_ROOT=/app

WORKDIR /app

# Copy only the requirements file first to keep this layer cacheable.
COPY web/requirements.txt /app/web/requirements.txt
RUN pip install -r /app/web/requirements.txt

# Copy the rest of the repo (web/, src/, drivers/, cars_csv/, tracks_csv/, setups/, samples/).
COPY . /app

EXPOSE 8080

CMD ["uvicorn", "web.main:app", "--host", "0.0.0.0", "--port", "8080"]
