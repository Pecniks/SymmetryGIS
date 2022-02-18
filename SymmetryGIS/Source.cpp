﻿#include <pch.h>
#include "RibbonUI.h"
#include <UxTheme.h>
#include <filesystem>
#include <vector>

#pragma comment(lib, "version.lib")
#pragma comment (lib, "UxTheme.lib")
using namespace GemmaFusion;

/**
 * @brief Georeferenced canvas helpers
*/
struct CanvasHelper
{
	CanvasHelper(Camera camera, WorldGeoreferencer* georeferencer, ShapeDrawing drawing)
		: m_Camera(camera)
		, m_Georeferencer(georeferencer)
		, m_ShapeDrawing(drawing)
	{
	}

	void PanToPoint(const Point3d& point)
	{
		BoundingBox2d bb{point - 10, point + 10};
		SetViewport(bb, true);
	}
	void ZoomToShapeLayer(ShapeLayer layer, bool noZoom)
	{
		auto bb = layer->GetBoundingBox();
		if(bb.Max >= bb.Min)
		{
			SetViewport(bb, noZoom);
		}
	}
	void ZoomToShapeObject(ShapeObject pObject, bool noZoom)
	{
		BoundingBox2d bb;
		if(pObject->CountPoints(PointCountType::Both) > 0)
		{
			pObject->GetBoundingBox(bb);
			if(bb.Min == bb.Max)
			{
				noZoom = true;
			}
			SetViewport(bb, noZoom);
		}
	}
	void ZoomToBoundingBox(const BoundingBox2d& bb)
	{
		if(bb.Max >= bb.Min)
		{
			SetViewport(bb, false);
		}
	}

	/**
	 * @brief Recenter camera to the current location
	*/
	void RecenterGrid()
	{
		Point3d pt{0.0, 0.0, 0.0};
		auto pos = m_Camera->GetPosition2();
		double z = pos.Z;

		pt = m_Georeferencer->WorldToGeoreferencedCoordinates(pos);
		pt.Z = 0;

		m_Georeferencer->SetBasePoint(pt);
		m_Camera->SetPosition(0, 0, -512);
		m_Camera->SetZ(static_cast<float>(z));
		m_ShapeDrawing->Redraw();
	}

	/**
	 * @brief Get georeferenced location at camera center
	 * @return
	*/
	Point3d GetCameraGeoreferencedCoordinates()
	{
		auto cameraPosition = m_Camera->GetPosition2();
		auto georeferencedPosition = m_Georeferencer->WorldToGeoreferencedCoordinates(cameraPosition);
		return georeferencedPosition;
	}

private:

	void SetViewport(const BoundingBox2d& GeoreferencedBoundingBox, bool noZoom)
	{
		Point3d lt = {GeoreferencedBoundingBox.Min.X, GeoreferencedBoundingBox.Max.Y, 0.0f};
		Point3d rb = {GeoreferencedBoundingBox.Max.X, GeoreferencedBoundingBox.Min.Y, 0.0f};

		Point3d wlt;
		Point3d wrb;

		m_Georeferencer->GeoreferencedToWorldCoordinates(&lt, &wlt);
		m_Georeferencer->GeoreferencedToWorldCoordinates(&rb, &wrb);

		float cz = 0.0f;
		if(noZoom)
		{
			cz = m_Camera->GetPosition().z;
		}

		GF::Renderers::GraphicsEngine::RECTD rc = {wlt.X, wlt.Y, wrb.X, wrb.Y};
		m_Camera->SetViewportOrthogonal(rc);

		if(noZoom)
		{
			m_Camera->SetZ(cz); // GetPositionLocal().z = cz;
		}
		m_Camera->Update();

		RecenterGrid();
	}


	Camera m_Camera;
	Core::Ptr<WorldGeoreferencer> m_Georeferencer;
	ShapeDrawing m_ShapeDrawing;
};

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE, LPSTR cmdArgs, int nShowWnd)
{
	//
	// Init application
	//

	// Init logger (debug output)
	AppLogger()->SetLogLevel(0);
	AppLogger()->EnableLine(true);// Output line
	AppLogger()->EnableTime(true);// Output line
	AppLogger()->EnableFunction(true);// Output function name
	AppLogger()->EnableLogging(true);
	

	APPRESULT_INFORMATION("SymmetryGIS %d", 6);

	// Init
	winrt::init_apartment(winrt::apartment_type::single_threaded);		// Init COM apartment
	SetThreadPerMonitorDpiAwarenessContext();							// This application is HiDPI aware and can handle scaling on its own
	g_ApplicationState.GraphicsProfile = GraphicsProfile::HighQuality;  // Global variable to toggle graphics quality (will be removed in future releases)
	InitializeFrequent();												// Init legacy STEZA "security" layer which decodes commonly used constants used by camera and renderers (DEV MODE)

	ApplicationInstance applicationInstance(hInstance);	// Current application instance with event loop
	Window mainWindow(L"Symmetry GIS", 1300, 700);		// Main window
	
	// Init main spatial projection and transformations
	Epsg epsgStore;
	Crs wgs84 = epsgStore.GetCrs(4326);
	Crs utm32n = epsgStore.GetCrs(3794);
	Crs webMercator = epsgStore.GetCrs(3857);
	CrsTransformation geographicTransform(utm32n, wgs84);
	CrsTransformation projectedTransform(wgs84, utm32n);

	GdiPlusInstance gdiPlusInstance; // Init GDI+ (Required for png, jpg which are primarly used in MapLayerRenderer)
	
	//
	// Init UI 
	// 
	
	// Riborn menu
	RibbonFramework ribbon(mainWindow);
	EmptyLayout ribbonLayout;

	// Canvas (Main data drawing)
	GraphicsInfrastructure gi;
	GraphicsDevice9 deviceD3D9;	// D3D9 graphics device needed for canvas
	GraphicsDevice11 deviceD3D11 = gi.CreateDevice(::Graphics::DebugDevice::None, false); // D3D11 graphics device needed for canvas
	CanvasControl canvas(deviceD3D9, deviceD3D11, mainWindow);	// 2D Canvas for displaying shapes, rasters and lidar
	
	// Status bar
	StatusBar statusBar(mainWindow->GenerateCmdId(), mainWindow);
	statusBar->AddLabel("projection", 350, std::format("🌐 EPSG:{} ({})", utm32n.GetEpsg(), utm32n.GetName()));

	// Prepare main layout for window (binds WM_SIZE handler to the main window)
	StackLayout mainLayout(mainWindow, LayoutType::Vertical);
	ribbon->SetOnViewChange([&]() {
		mainLayout->ModifyLayout(ribbonLayout, UI::SizeType::FixedPixels, ribbon->GetRibbonHeight());
	});
	mainLayout->AddLayout(ribbonLayout); // Top ribbon
	mainLayout->AddControl(canvas);  // Canvas will use rest of the free space
	mainLayout->AddControl(statusBar, SizeType::FixedDips, 25); // Bottom control (statusbar) will have height of 25 dips




	//bool enableContext = true;
	//
	//auto l_enabled = [&]() -> bool
	//{
	//	return !enableContext;
	//};

	//auto l_invalidate_props = [&]
	//{
	//	//ribbon->InvalidateProperty(IDC_CMD_HOME_BUTTON);
	//	//ribbon->InvalidateProperty(IDC_CMD_HOME_COLOR);
	//	//ribbon->InvalidateProperty(IDC_CMD_HOME_CHECKBOX);
	//	//ribbon->InvalidateProperty(IDC_CMD_HOME_TEXT);
	//	//ribbon->InvalidateProperty(IDC_CMD_HOME_NUMERIC);
	//	//ribbon->InvalidateProperty(IDC_CMD_TAB_CONTEXT_GROUP);
	//};


	//
	// Init rendering and containers
	//
	
	// Get 3D camera from canvas
	Camera camera = canvas->GetCamera(); 
	camera->SetZ(-300000.0f);

	// Setup georeferencer (transforms georeferenced points into 3D world space)
	auto georeferencer = Core::make_ptr<WorldGeoreferencer>(camera);
	georeferencer->SetBaseScale({ 0.5, -0.5, -0.5 });
	georeferencer->SetBasePoint({ 1741650, 5870027 });

	// Prepare shape renderer
	auto shapeRenderer = Core::make_ptr<ShapeRenderer>(deviceD3D9, camera, georeferencer);
	shapeRenderer->SetDpi(mainWindow->GetDpi());
	// Prepare ShapeLayer container and assign renderers. ShapeLayer is automatically added to all renderers when inserted into a drawing
	ShapeDrawing shapeDrawing;
	shapeDrawing->Initialize({ shapeRenderer });
	shapeDrawing->SetTargetCrs(utm32n);
	// Prepare shape loader which prepares shapelayer for rendering in the background
	ShapeLoader shapeLoader;
	shapeLoader->AddLayerRenderer(shapeRenderer);
	shapeLoader->Start();

	// Prepare raster renderer
	auto rasterRenderer = Core::make_ptr<MonoRasterLayerRenderer>(deviceD3D9, camera, georeferencer);
	// Prepare RasterLayer container and assign renderers. RasterLayer is automatically added to all renderers when inserted into a drawing	
	RasterDrawing rasterDrawing;
	rasterDrawing->Initialize({ rasterRenderer });
	rasterDrawing->SetTargetCrs(utm32n);

	// Prepare map renderer
	MapLayerRenderer mapRenderer(deviceD3D9, camera, georeferencer);
	mapRenderer->EnableTextureFiltering(true);
	mapRenderer->SwapRedBlueColors(true); // 
	mapRenderer->SetTargetCrs(utm32n);

	// LasContainer is mixture of ShapeLayer and RasterLayer (uses both of them to achieve 2D rendering)
	LasContainer lasContainer;
	lasContainer->SetTargetCrs(utm32n);
	// Add las container shapelayer to the renderer and set it as support layer
	ShapeLayer lasShapeLayer = lasContainer->GetLasLayer();
	lasShapeLayer->GetStyle().FillSymbol().Show(false); // Disable shape fill
	shapeRenderer->CreateLayerObject(lasShapeLayer);

	TmsFetcher basemapFetcher("https://server.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/{z}/{y}/{x}.png");
	MapLayer basemapObject = mapRenderer->CreateLayerObject(basemapFetcher.as<TileFetcher>());

	// Current thread work dispatcher (captures current thread via RAII)
	ThreadDispatcher uiThread;
	// Prepare thread for background processing (lidar rendering or anything else)
	ThreadWorker processingThread("Processing thread", new_thread, Platform::Infinite());
	
	CanvasHelper canvasHelper(camera, georeferencer, shapeDrawing);

	//
	// Prepare initial data for visualization
	//
	
	// Define new custom colormap which will perform hillshade operation on lidar and sample raster object
	auto l_hillshade_colormap = [](Core::Ptr<GF::Spatial::Raster::impl::RasterObject> object, TiffTools::TiffImage<TiffTools::PixelDataRGBA8> outputImage) mutable
	{
		TiffTools::TiffImage<float> image = object->GetImage();
		auto grid = GF::Extensions::Interop::GetGrid(object.Get());
		grid.ApplyHillShade(45.0, 315.0);

		for(int y = 0; y < image.GetHeight(); y++)
		{
			for(int x = 0; x < image.GetWidth(); x++)
			{
				Core::Byte value = static_cast<Core::Byte>(grid.GetValue(x, y) * 255.0);
				TiffTools::PixelDataRGBA8 rgba{value, value, value, 255};
				outputImage.SetPixel(x, y, rgba);
			}
		}
	};

	//reading symetrie file
	auto readSymetrieFile = [](std::string filename, Utils::Plane& plane) mutable
	{
		std::ifstream file(filename);
		//file.open(filename, std::ios::in);

		if(!file.is_open())
		{
			throw std::exception("File not opened!");
			return;
		}

		std::string line;
		std::vector<double> list;
		while(std::getline(file, line, ';'))
		{
			list.push_back(atof(line.c_str()));
		}

		plane.a = (float)list[0];
		plane.b = (float)list[1];
		plane.c = (float)list[2];
		plane.d = (float)list[3];

		file.close();
	};

	//
	// Event handlers
	//
	canvas->SetDrawEvent([&](Camera camera)
	{
		deviceD3D9->GetD3D9Device()->SetRenderState(D3DRS_ZENABLE, FALSE);// Disable ZBuffer
		// Render layers
		mapRenderer->Draw();
		rasterRenderer->Draw();
		shapeRenderer->Update();
		shapeRenderer->Draw();
		return true;
	});

	//auto lastUpdateStatus = std::chrono::high_resolution_clock::now();
	canvas->SetMouseEvent(MouseEventType::MouseMove, [&](const Mouse& mouse) {
		auto world = camera->ScreenToWorldCoordinates(mouse.CurrentPosition2());
		auto geo = georeferencer->WorldToGeoreferencedCoordinates(world.cast<Point3d>());
		geo.Z = 0;

		//auto updateStatus = std::chrono::high_resolution_clock::now();
		//if(std::chrono::duration_cast<std::chrono::milliseconds>(updateStatus - lastUpdateStatus).count() > 64)
		//{
		//	auto latLon = geographicTransform.TransformCoordinate(geo);

		//	// This file is saved as UTF-8 with BOM which means utf8 is supported by GF Apis (std::string)
		//	statusBar->SetItemText("coordinates", Core::String::Format("%.2f m\n%.2f m", geo.X, geo.Y));
		//	statusBar->SetItemText("longlat", Core::String::Format("%f°\n%f°", latLon.X, latLon.Y));
		//	statusBar->SetItemText("zoom", Core::String::Format("🔎 %.2f", abs(camera->GetPosition2().Z)));

		//	lastUpdateStatus = updateStatus;
		//}
		shapeRenderer->Update(true);
	});

	canvas->SetMouseEvent(MouseEventType::LButtonDown, [&](const Mouse& mouse) {
		canvas->SetCursor(MouseCursor::SizeAll);
	});

	canvas->SetMouseEvent(MouseEventType::LButtonUp, [&](const Mouse& mouse) {
		canvas->SetCursor(MouseCursor::Arrow);
	});

	canvas->SetMouseEvent(MouseEventType::RButtonUp, [&](const Mouse& mouse) {
		ContextMenu canvasContextMenu;
		auto [x, y] = canvas->GetScreenCoordinates(static_cast<int>(mouse.CurrentPosition().x), static_cast<int>(mouse.CurrentPosition().y));
		canvasContextMenu->Open(canvas, x, y);
	});

	canvas->SetMouseWheelEvent([&](const Mouse& mouse, int zDelta) {
		shapeRenderer->Update(true);
		//auto updateStatus = std::chrono::high_resolution_clock::now();
		//if(std::chrono::duration_cast<std::chrono::milliseconds>(updateStatus - lastUpdateStatus).count() > 64)
		//{
		//	statusBar->SetItemText("zoom", Core::String::Format("🔎 %.2f", abs(camera->GetPosition2().Z)));
		//	lastUpdateStatus = updateStatus;
		//}
	});

	mainWindow->AddEventHandler(WM_CLOSE, [&](HWND hWnd, WPARAM wParam, LPARAM lParam, LRESULT& lResult)
	{
		applicationInstance->Exit(0);
		return true;
	});

	mainWindow->AddEventHandler(WM_DPICHANGED, [&](HWND hWnd, WPARAM wParam, LPARAM lParam, LRESULT& lResult)
	{
		shapeRenderer->SetDpi(mainWindow->GetDpi());
		return false;
	});

	// Button click
	ribbon->SetOnCommandExecute(ID_EXIT, [&]()
	{
		//APPRESULT_DEV_INFORMATION("Exit");
		applicationInstance->Exit(0);
	});
	
	ribbon->SetOnCommandExecute(ID_CLEARALL,[&]{
			shapeDrawing->Clear();
			rasterDrawing->Clear();
			lasContainer->Clear();
	});

	ribbon->SetOnCommandExecute(ID_SWITCHPROJ, [&]{
		CrsPickerWindow crsPicker;
		if(auto epsgCode = crsPicker.GetEpsg(true))
		{
			Crs oldCrs = shapeDrawing->GetTargetCrs();
			Crs newCrs = epsgStore.GetCrs(epsgCode.value());
			if(newCrs.GetType() != CrsType::Projected)
			{
				MessageBoxW(0, L"Only projected crs types are supported", L"Error", MB_ICONERROR);
				return;
			}
			shapeDrawing->SetTargetCrs(newCrs);
			rasterDrawing->SetTargetCrs(newCrs);
			mapRenderer->SetTargetCrs(newCrs);
			lasContainer->SetTargetCrs(newCrs);
			projectedTransform = CrsTransformation(wgs84, newCrs);
			geographicTransform = CrsTransformation(newCrs, wgs84);
			statusBar->SetItemText("projection", std::format("🌐 EPSG:{} ({})", newCrs.GetEpsg(), newCrs.GetName()));

			// Retransform current location
			CrsTransformation transformation(oldCrs, newCrs);
			auto currentGeoreferenced = canvasHelper.GetCameraGeoreferencedCoordinates();
			currentGeoreferenced.Z = 0;
			auto newGeoreferenced = transformation.TransformCoordinate(currentGeoreferenced);
			canvasHelper.PanToPoint(newGeoreferenced);
			canvasHelper.RecenterGrid();
		}
	});

	ribbon->SetOnCommandExecute(ID_LOADDATA, [&]{
		APPRESULT_DEV_INFORMATION("Load Data");
	FilePicker picker;
	//auto las = picker.AddExtensionFilter("LiDAR files", "*.las");
	//if(auto result = picker.OpenMultipleDialog("Open lidar files"))
	//{
	//	std::vector<ShapeObject> added;
	//	std::vector<Utils::Plane> planes;
	//	for(auto& file : result.value().Path())
	//	{
	//		added.push_back(lasContainer->AddLas(file));

	//		// create point cloud from las files
	//		auto args = fmt::format("las2txt.exe -i {} --column-delimiter \" \" --line-format x y z -o tmp\\", file);
	//		auto processInstance = Platform::Windows::Process::CreateProcessInstance("las2txt\\las2txt.exe", args);
	//		processInstance->Start();
	//		processInstance->Await();
	//		auto pos = file.find_last_of("\\");
	//		std::string oldf(file, pos + 1); oldf.insert(0, "tmp\\"); oldf.replace(oldf.size() - 3, 3, "txt");
	//		std::string newf(file, pos + 1); newf.insert(0, "tmp\\"); newf.replace(newf.size() - 3, 3, "pc");
	//		std::filesystem::rename(oldf.c_str(), newf.c_str());


	//		auto argsym = fmt::format("SymmetryDetector.exe {} {}", newf, false);
	//		auto processInstanceSym = Platform::Windows::Process::CreateProcessInstance("SymmetryDetector\\SymmetryDetector.exe", argsym);
	//		processInstanceSym->Start();//std::format()
	//		processInstanceSym->Await();
	//		Utils::Plane plane;
	//		readSymetrieFile(newf + ".txt", plane);
	//		planes.push_back(plane);

	//		auto bb = added.back()->GetBoundingBox2d();
	//		Utils::BoundingBox6f BB((float)bb.Min.X, (float)bb.Min.Y, 0.f, (float)bb.Max.X, (float)bb.Max.Y, 0.f);

	//		Utils::Plane P = Utils::Plane(BB.GetBottomBackLeft(), BB.GetBottomBackRight(), BB.GetBottomFrontLeft());
	//		auto ray = Utils::Intersects(plane, P);
	//		float kao = Utils::DotProd(BB.Min + ray.o, ray.d);

	//		auto o = ray.o + ray.d * kao;
	//		ray.SetLength(100);

	//		ShapeLayer lineLayer(SpecificGeometryType::LineStringZ);
	//		//lineLayer->SetSourceCrs(utm32n);
	//		//lineLayer->SetTargetCrs(utm32n);
	//		lineLayer->CreateShapeObject({{ ray.o.x, ray.o.y, ray.o.z + 100 }, { ray.GetSecondPoint(100).x, ray.GetSecondPoint(100).y, ray.GetSecondPoint(100).z}});
	//		lineLayer->GetStyle().SetColor({1.0f, 0.0f, 0.0f, 1.0f}).PointSymbol().SetSize(5.0f).GetStyle().OutlineSymbol().SetSize(5.0f);
	//		shapeDrawing->AddShapeLayer(lineLayer);
	//		canvasHelper.ZoomToShapeLayer(lineLayer, false);
	//		//applicationInstance->GetWorker().Post([&] { canvasHelper.ZoomToShapeLayer(lineLayer, false); });
	//	}

	//	// Submit lidar rendering to background thread
	//	processingThread.Post([added, uiThread, lasContainer, rasterDrawing, l_hillshade_colormap]() mutable
	//						{
	//							for(auto& addedLas : added)
	//							{
	//								auto raster = lasContainer->CreateLasRasterLayer(addedLas);
	//								lasContainer->GenerateLasRasterObjects(raster, 1.0, GenerateLasRasterObjectProperties(true, true, {2, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18}));
	//								// Submit rendered raster to the ui thread
	//								uiThread->Post([raster, rasterDrawing, l_hillshade_colormap]() mutable
	//												{
	//													raster[0]->SetCustomColorMapKey("Hillshade");
	//													raster[0]->SetCustomColorMapHandler(l_hillshade_colormap);
	//													rasterDrawing->AddRasterLayer(raster);
	//												});
	//							}
	//						});

	//	//canvasHelper.ZoomToBoundingBox(added.at(0)->GetBoundingBox2d());

	//}
	});

	//
	// Show main window and run event loop
	//
	ribbon->LoadUI();	// Load ribbon UI after everything is set
	mainWindow->Show(SW_SHOW);	// Display window
	mainLayout->Recalculate();
	int returnCode = applicationInstance->Run(0, nullptr); 	// Run main event loop
	shapeLoader->Stop(); 	// Cleanup
	return returnCode;
}