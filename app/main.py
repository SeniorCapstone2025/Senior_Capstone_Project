from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.routes import commands, status, detections, inventory, camera, websocket_route
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

# Configure CORS middleware - temporarily allow all origins for debugging
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Temporarily allow all - will restrict later
    allow_credentials=False,  # Must be False when using allow_origins=["*"]
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS", "PATCH"],
    allow_headers=["*"],
    expose_headers=["*"],
    max_age=3600,
)

# Include routers
app.include_router(commands.router)
app.include_router(status.router)
app.include_router(detections.router)
app.include_router(inventory.router)
app.include_router(camera.router)
app.include_router(websocket_route.router)

@app.get("/")
async def root():
    return {"message": "Rover is running properly 👨🏾‍💻"}


@app.get("/test")
async def test():
    """Test endpoint"""
    return {"test": "working"}


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "app": settings.app_name}