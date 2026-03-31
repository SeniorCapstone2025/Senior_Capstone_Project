from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.routes import auth, commands, status, detections, websocket_route, inventory
# from app.routes import camera  # TODO: Enable when zbar is properly configured
from app.config import get_settings
from app.rosbridge import rosbridge_client
from app.ros_handlers import setup_ros_handlers
import logging
from datetime import datetime
import time

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

settings = get_settings()

# Track application startup time
app_start_time = time.time()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage application lifecycle - connect/disconnect rosbridge."""
    # Startup
    logger.info("Starting rosbridge connection...")
    rosbridge_client.url = settings.rosbridge_url
    try:
        
        await rosbridge_client.connect()
        await setup_ros_handlers()
    except Exception as e:
        logger.error(f"Error during rosbridge connection: {e}")

    yield

    # Shutdown
    logger.info("Shutting down rosbridge connection...")
    await rosbridge_client.disconnect()


app = FastAPI(title=settings.app_name, lifespan=lifespan)

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
app.include_router(auth.router)
app.include_router(commands.router)
app.include_router(status.router)
app.include_router(detections.router)
# app.include_router(camera.router)  # TODO: Enable when zbar is properly configured
app.include_router(inventory.router)
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
    """Health check endpoint with detailed system information"""
    uptime = time.time() - app_start_time
    return {
        "status": "ok",
        "app": settings.app_name,
        "version": "2.0.0",
        "timestamp": datetime.now().isoformat(),
        "uptime": uptime,
        "rosbridge_connected": rosbridge_client.is_connected
    }