from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.routes import auth, commands, status, detections, camera, websocket_route, schedule as schedule_route
from app.config import get_settings
from app.rosbridge import rosbridge_client
from app.ros_handlers import setup_ros_handlers
from app.scheduler import start_scheduler, stop_scheduler
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

settings = get_settings()


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

    start_scheduler()

    yield

    # Shutdown
    stop_scheduler()
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
app.include_router(camera.router)
app.include_router(websocket_route.router)
app.include_router(schedule_route.router)

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