"""/api/sim, /api/solve, /api/validate, /api/sim/result/... routes."""

from __future__ import annotations

import io
import logging

from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse

from . import jobs, sim_runner

logger = logging.getLogger(__name__)
router = APIRouter()


@router.post("/api/sim")
async def post_sim(req: dict):
    try:
        return await sim_runner.run_sim(req)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e
    except (ValueError, KeyError) as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    except Exception as e:  # noqa: BLE001
        logger.exception("Sim failed")
        raise HTTPException(status_code=500, detail=f"{type(e).__name__}: {e}") from e


@router.post("/api/solve")
async def post_solve(req: dict):
    try:
        return await sim_runner.run_solve(req)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e
    except (ValueError, KeyError) as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    except Exception as e:  # noqa: BLE001
        logger.exception("Solve failed")
        raise HTTPException(status_code=500, detail=f"{type(e).__name__}: {e}") from e


@router.post("/api/validate")
async def post_validate(req: dict):
    try:
        return await sim_runner.run_validate(req)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e
    except (ValueError, KeyError) as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    except Exception as e:  # noqa: BLE001
        logger.exception("Validate failed")
        raise HTTPException(status_code=500, detail=f"{type(e).__name__}: {e}") from e


def _csv_response(df, filename: str) -> StreamingResponse:
    """Return a DataFrame as a streaming text/csv response."""
    if df is None:
        raise HTTPException(status_code=404, detail=f"{filename}: no data on job")
    buf = io.StringIO()
    df.to_csv(buf, index=False)
    body = buf.getvalue().encode("utf-8")
    return StreamingResponse(
        iter([body]),
        media_type="text/csv",
        headers={"Content-Disposition": f'inline; filename="{filename}"'},
    )


@router.get("/api/sim/result/{job_id}/telemetry.csv")
async def get_telemetry_csv(job_id: str):
    try:
        job = jobs.require(job_id)
    except KeyError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e
    return _csv_response(job.telemetry_df, "telemetry.csv")


@router.get("/api/sim/result/{job_id}/trace.csv")
async def get_trace_csv(job_id: str):
    try:
        job = jobs.require(job_id)
    except KeyError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e
    return _csv_response(job.trace_df, "trace.csv")


@router.get("/api/sim/result/{job_id}/stint_summary.csv")
async def get_stint_summary_csv(job_id: str):
    try:
        job = jobs.require(job_id)
    except KeyError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e
    return _csv_response(job.stint_summary_df, "stint_summary.csv")


@router.get("/api/sim/result/{job_id}/validation_bins.csv")
async def get_validation_bins_csv(job_id: str):
    try:
        job = jobs.require(job_id)
    except KeyError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e
    return _csv_response(job.validation_bins_df, "validation_bins.csv")


@router.get("/api/sim/result/{job_id}/plot.png")
async def get_plot_png(job_id: str):
    try:
        job = jobs.require(job_id)
    except KeyError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e
    if job.plot_png is None:
        raise HTTPException(
            status_code=404,
            detail="No matplotlib plot was rendered for this job (Plotly used instead).",
        )
    return StreamingResponse(iter([job.plot_png]), media_type="image/png")


@router.get("/api/sim/result/{job_id}/overlay.png")
async def get_overlay_png(job_id: str):
    try:
        job = jobs.require(job_id)
    except KeyError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e
    if job.overlay_png is None:
        raise HTTPException(
            status_code=404,
            detail="No overlay image rendered for this job.",
        )
    return StreamingResponse(iter([job.overlay_png]), media_type="image/png")
