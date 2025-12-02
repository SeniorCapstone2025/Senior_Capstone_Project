from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.routes import commands, status, detections, inventory
from app.config import get_settings
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

settings = get_settings()

app = FastAPI(title=settings.app_name)

# Configure CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(commands.router)
app.include_router(status.router)
app.include_router(detections.router)
app.include_router(inventory.router)

@app.get("/")
async def root():
    return {"message": "Rover is running properly 👨🏾‍💻"}


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "app": settings.app_name}


