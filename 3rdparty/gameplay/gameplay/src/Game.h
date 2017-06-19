#pragma once

#include "Rectangle.h"
#include "RenderState.h"

#include "gl/pixel.h"

#include <GLFW/glfw3.h>

#include <chrono>


namespace gameplay
{
    class Scene;
    class RenderContext;


    /**
     * Defines the base class your game will extend for game initialization, logic and platform delegates.
     *
     * This represents a running cross-platform game application and provides an abstraction
     * to most typical platform functionality and events.
     *
     * @see http://gameplay3d.github.io/GamePlay/docs/file-formats.html#wiki-Game_Config
     */
    class Game
    {
        friend class ShutdownListener;

    public:

        /**
         * The game states.
         */
        enum State
        {
            UNINITIALIZED,
            RUNNING,
            PAUSED
        };


        /**
         * Flags used when clearing the active frame buffer targets.
         */
        enum ClearFlags
        {
            CLEAR_COLOR = GL_COLOR_BUFFER_BIT,
            CLEAR_DEPTH = GL_DEPTH_BUFFER_BIT,
            CLEAR_COLOR_DEPTH = CLEAR_COLOR | CLEAR_DEPTH
        };


        /**
         * Constructor.
         */
        explicit Game();

        /**
         * Destructor.
         */
        virtual ~Game();

        /**
         * Gets whether vertical sync is enabled for the game display.
         *
         * @return true if vsync is enabled; false if not.
         */
        bool isVsync() const;

        /**
         * Sets whether vertical sync is enabled for the game display.
         *
         * @param enable true if vsync is enabled; false if not.
         */
        void setVsync(bool enable);

        /**
         * Gets the total game time (in milliseconds). This is the total accumulated game time (unpaused).
         *
         * You would typically use things in your game that you want to stop when the game is paused.
         * This includes things such as game physics and animation.
         *
         * @return The total game time (in milliseconds).
         */
        std::chrono::high_resolution_clock::time_point getGameTime() const
        {
            return std::chrono::high_resolution_clock::now() - _pausedTimeTotal - _constructionTime.time_since_epoch();
        }

        /**
         * Gets the game state.
         *
         * @return The current game state.
         */
        inline State getState() const;

        /**
         * Determines if the game has been initialized.
         *
         * @return true if the game initialization has completed, false otherwise.
         */
        inline bool isInitialized() const;

        /**
         * Called to initialize the game, and begin running the game.
         *
         * @return Zero for normal termination, or non-zero if an error occurred.
         */
        int run();

        /**
         * Pauses the game after being run.
         */
        void pause();

        /**
         * Resumes the game after being paused.
         */
        void resume();

        /**
         * Exits the game.
         */
        void exit();

        /**
         * Platform frame delegate.
         *
         * This is called every frame from the platform.
         * This in turn calls back on the user implemented game methods: update() then render()
         */
        void frame();

        /**
         * Gets the current frame rate.
         *
         * @return The current frame rate.
         */
        inline unsigned int getFrameRate() const;

        /**
         * Gets the game window width.
         *
         * @return The game window width.
         */
        inline unsigned int getWidth() const;

        /**
         * Gets the game window height.
         *
         * @return The game window height.
         */
        inline unsigned int getHeight() const;

        /**
         * Gets the aspect ratio of the window. (width / height)
         *
         * @return The aspect ratio of the window.
         */
        inline float getAspectRatio() const;

        /**
         * Gets the game current viewport.
         *
         * The default viewport is Rectangle(0, 0, Game::getWidth(), Game::getHeight()).
         */
        inline const Rectangle& getViewport() const;

        /**
         * Sets the game current viewport.
         *
         * The x, y, width and height of the viewport must all be positive.
         *
         * viewport The custom viewport to be set on the game.
         */
        void setViewport(const Rectangle& viewport);

        /**
         * Clears the specified resource buffers to the specified clear values.
         *
         * @param flags The flags indicating which buffers to be cleared.
         * @param clearColor The color value to clear to when the flags includes the color buffer.
         * @param clearDepth The depth value to clear to when the flags includes the color buffer.
         */
        void clear(ClearFlags flags, const gl::RGBA8& clearColor, float clearDepth);

        /**
         * Clears the specified resource buffers to the specified clear values.
         *
         * @param flags The flags indicating which buffers to be cleared.
         * @param red The red channel.
         * @param green The green channel.
         * @param blue The blue channel.
         * @param alpha The alpha channel.
         * @param clearDepth The depth value to clear to when the flags includes the color buffer.
         */
        void clear(ClearFlags flags, uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha, float clearDepth);

        /**
         * Sets whether multi-sampling is to be enabled/disabled. Default is disabled.
         */
        inline void setMultiSampling(unsigned samples);

        /**
         * Is multi-sampling enabled.
         *
         * @return true if multi-sampling is enabled.
         */
        inline unsigned getMultiSampling() const;

        bool loop()
        {
            glfwPollEvents();

            return glfwWindowShouldClose(_window) == GL_FALSE;
        }


        /**
        * Renders a single frame once and then swaps it to the display.
        * This calls the given script function, which should take no parameters and return nothing (void).
        *
        * This is useful for rendering splash screens.
        */
        void swapBuffers();


        GLFWwindow* getWindow() const
        {
            return _window;
        }


        const std::shared_ptr<Scene>& getScene() const
        {
            return _scene;
        }

    protected:

        /**
         * Render callback for handling rendering routines.
         *
         * Called just after update, once per frame when game is running.
         * Ideal for all rendering code.
         */
        virtual void render(bool wireframe = false);

    private:
        Game(const Game& copy) = delete;

        /**
         * Starts the game.
         */
        bool startup();

        /**
         * Shuts down the game.
         */
        void shutdown();

        bool _initialized = false; // If game has initialized yet.
        State _state = UNINITIALIZED; // The game state.
        const std::chrono::high_resolution_clock::time_point _constructionTime{std::chrono::high_resolution_clock::now()};
        std::chrono::high_resolution_clock::time_point _pauseStart{};
        std::chrono::high_resolution_clock::duration _pausedTimeTotal{0}; // The total time paused.
        std::chrono::high_resolution_clock::time_point _frameLastFPS{}; // The last time the frame count was updated.
        unsigned int _frameCount = 0; // The current frame count.
        unsigned int _frameRate = 0; // The current frame rate.
        int _width = 0; // The game's display width.
        int _height = 0; // The game's display height.
        Rectangle _viewport; // the games's current viewport.
        gl::RGBA8 _clearColor; // The clear color value last used for clearing the color buffer.
        float _clearDepth = 1; // The clear depth value last used for clearing the depth buffer.

        bool _vsync = WINDOW_VSYNC;
        unsigned _multiSampling = 0;
        GLFWwindow* _window = nullptr;

        std::shared_ptr<Scene> _scene;

        friend class ScreenDisplayer;
    };


    inline Game::State Game::getState() const
    {
        return _state;
    }


    inline bool Game::isInitialized() const
    {
        return _initialized;
    }


    inline unsigned int Game::getFrameRate() const
    {
        return _frameRate;
    }


    inline unsigned int Game::getWidth() const
    {
        return _width;
    }


    inline unsigned int Game::getHeight() const
    {
        return _height;
    }


    inline float Game::getAspectRatio() const
    {
        return static_cast<float>(_width) / static_cast<float>(_height);
    }


    inline const Rectangle& Game::getViewport() const
    {
        return _viewport;
    }


    inline void Game::setMultiSampling(unsigned samples)
    {
        //! @todo Really enable multisampling
        _multiSampling = samples;
    }


    inline unsigned Game::getMultiSampling() const
    {
        return _multiSampling;
    }
}
